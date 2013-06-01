/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <termios.h>
#include <argp.h>
#include <somatic.h>
#include <somatic/daemon.h>
#include "fastrak.h"

#define FREQ_MAX 100

#define KEY_PEEKABOT_HOST 300

typedef struct {
    somatic_d_t d;
    somatic_d_opts_t d_opts;

    // Option Vars
    const char *opt_chan_name;
    double opt_freq;
    int opt_peekabot;
    const char *opt_peekabot_host;
    const char *opt_device;
    int opt_kalman;
    double opt_diag_E;
    double opt_diag_Q;
    double opt_diag_R;

    Somatic__MultiTransform *msg;
    ach_channel_t chan;
    fastrak_t fk;
} cx_t;


/* ---------- */
/* ARGP Junk  */
/* ---------- */

static struct argp_option options[] = {
    {
        .name = "verbose",
        .key = 'v',
        .arg = NULL,
        .flags = 0,
        .doc = "Causes verbose output"
    },
    {
        .name = "frequency",
        .key = 'f',
        .arg = "frequency",
        .flags = 0,
        .doc = "Approx. frequency to transmit images, Hz"
    },
    {
        .name = "peekabot",
        .key = 'P',
        .arg = NULL,
        .flags = 0,
        .doc = "Causes data to be plotted via peekabot"
    },
    {
        .name = "port",
        .key = 'p',
        .arg = "tty",
        .flags = 0,
        .doc = "TTY to use (default /dev/ttyS0)"
    },
    {
        .name = "peekabot-host",
        .key = KEY_PEEKABOT_HOST,
        .arg = "hostname",
        .flags = 0,
        .doc = "hostname of the peekabot"
    },
    {
        .name = "chan",
        .key = 'c',
        .arg = "ach channel",
        .flags = 0,
        .doc = "ach channel to send data to"
    },
    {
        .name = "kalman",
        .key = 'k',
        .arg = NULL,
        .flags = 0,
        .doc = "use the kalman filter"
    },
    {
        .name = "measurement-noise",
        .key = 'Q',
        .arg = "double",
        .flags = 0,
        .doc = "diagonal values of Q"
    },
    {
        .name = "process-noise",
        .key = 'R',
        .arg = "double",
        .flags = 0,
        .doc = "diagonal values of R"
    },
    {
        .name = "covariance",
        .key = 'E',
        .arg = "double",
        .flags = 0,
        .doc = "diagonal values of E"
    },
    SOMATIC_D_ARGP_OPTS,
    {
        .name = NULL,
        .key = 0,
        .arg = NULL,
        .flags = 0,
        .doc = NULL
    }
};


/// argp parsing function
static int parse_opt( int key, char *arg, struct argp_state *state);
/// argp program version
const char *argp_program_version = "fastrakd 0.0";
/// argp program arguments documention
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "Talks to fastrak sensor";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


static int parse_opt( int key, char *arg, struct argp_state *state) {
    cx_t *cx = (cx_t*)state->input;
    int r;
    switch(key) {
    case 'v':
        somatic_opt_verbosity++;
        break;
    case 'P':
        cx->opt_peekabot++;
        break;
    case KEY_PEEKABOT_HOST:
        cx->opt_peekabot_host = strdup(arg);
        break;
    case 'f':
        cx->opt_freq = atof(arg);
        break;
    case 'p':
        cx->opt_device = strdup(arg);
        break;
    case 'k':
        cx->opt_kalman++;
        break;
    case 'E':
        r = sscanf(arg, "%lf", &cx->opt_diag_E );
        somatic_hard_assert( 1 == r, "Couldn't parse arg: %s\n", arg );
        break;
    case 'Q':
        r = sscanf(arg, "%lf", &cx->opt_diag_Q );
        somatic_hard_assert( 1 == r, "Couldn't parse arg: %s\n", arg );
        break;
    case 'R':
        r = sscanf(arg, "%lf", &cx->opt_diag_R );
        somatic_hard_assert( 1 == r, "Couldn't parse arg: %s\n", arg );
        break;
    case 'c':
        if( strlen(arg) > ACH_CHAN_NAME_MAX ) {
            fprintf(stderr, "ERROR: channel is too long\n");
            exit(1);
        }else {
            cx->opt_chan_name = strdup( arg );
        }
        break;
    case 0:
        break;
    }

    somatic_d_argp_parse( key, arg, &cx->d_opts );

    return 0;
}

/* ------------- */
/* Program Code  */
/* ------------- */

void init(cx_t *cx) {
    somatic_d_init(&cx->d,
                   &cx->d_opts);  // init daemon variables, channels, log, etc

    // init fastrak
    int r = fastrak_init(&cx->fk, cx->opt_device );
    somatic_d_require( &cx->d, 0 == r,
                       "Couldn't open fastrak: %s", strerror(errno));

    // init msg
    cx->msg = somatic_multi_transform_alloc(4);

    somatic_d_channel_open( &cx->d, &cx->chan,
                            cx->opt_chan_name, NULL );
    somatic_verbprintf( 2, "Initialized\n");
}

void destroy(cx_t *cx) {
    // close up
    somatic_verbprintf( 1, "finishing\n", cx->opt_freq );
    fastrak_destroy(&cx->fk);
    somatic_d_channel_close( &cx->d, &cx->chan );
    somatic_d_destroy(&cx->d);
}

void dump(cx_t *cx) {
    for(int q=0; q<3; q++){
        printf("%f ", cx->fk.tf_vq[0][q]);
    }

    double s = cx->fk.tf_vq[0][3];
    double x = cx->fk.tf_vq[0][4];
    double y = cx->fk.tf_vq[0][5];
    double z = cx->fk.tf_vq[0][6];

    double sqw = s*s;
    double sqx = x*x;
    double sqy = y*y;
    double sqz = z*z;

    double rx = atan2(2.f * (x*y + z*s), sqx - sqy - sqz + sqw);
    double ry = asin(-2.f * (x*z - y*s));
    double rz = atan2(2.f * (y*z + x*s), -sqx - sqy + sqz + sqw);


    printf("%f %f %f\n", rx,ry,rz);

    /*} else {
      euler.x = atan2f(2.f * (v.z*v.y + v.x*s), 1 - 2*(sqx + sqy));
      euler.y = asinf(-2.f * (v.x*v.z - v.y*s));
      euler.z = atan2f(2.f * (v.x*v.y + v.z*s), 1 - 2*(sqy + sqz));
      }
      return euler;*/
    //printf("\n");
}

void run(cx_t *cx) {
    somatic_d_event( &cx->d, SOMATIC__EVENT__PRIORITIES__NOTICE,
                     SOMATIC__EVENT__CODES__PROC_RUNNING,
                     NULL, NULL );
    while(!somatic_sig_received) {
        // get tfs
        int r = fastrak_read(&cx->fk);
        somatic_verbprintf( 2, "Got reading\n");
        if( 0 == r ) {
            // fill protobuf with tfs
            for( size_t i = 0; i < 4; i ++ ) {
                somatic_transform_set_vec( cx->msg->tf[i], cx->fk.tf_vq[i] );
                somatic_transform_set_quat( cx->msg->tf[i], cx->fk.tf_vq[i] + 3 );
            }
            if( somatic_opt_verbosity >= 3 ) {
                dump(cx);
            }

            // set time
            somatic_metadata_set_time_now( cx->msg->meta );

            // send
            r = SOMATIC_PACK_SEND( &cx->chan, somatic__multi_transform, cx->msg );

        }
    }
    somatic_d_event( &cx->d, SOMATIC__EVENT__PRIORITIES__NOTICE,
                     SOMATIC__EVENT__CODES__PROC_STOPPING,
                     NULL, NULL );
}

int main( int argc, char **argv ) {

    static cx_t cx;
    memset(&cx, 0, sizeof(cx));
    cx.d_opts.ident = "fastrakd";
    cx.d_opts.sched_rt = SOMATIC_D_SCHED_UI; // logger not realtime

    cx.opt_chan_name = "fastrak";
    cx.opt_freq = -1;
    cx.opt_peekabot = 0;
    cx.opt_peekabot_host = "localhost";
    cx.opt_device = "/dev/ttyS0";
    cx.opt_kalman = 0;
    cx.opt_diag_E = 1;
    cx.opt_diag_Q = 250;
    cx.opt_diag_R = 1;


    // parse options
    argp_parse (&argp, argc, argv, 0, NULL, &cx);
    somatic_verbprintf_prefix="fastrak";
    somatic_verbprintf( 1, "Frequency of %d Hz\n", cx.opt_freq );

    init(&cx);
    run(&cx);
    destroy(&cx);

    return 0;
}

