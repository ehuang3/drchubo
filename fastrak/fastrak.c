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

#include <amino.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include "fastrak.h"

int fastrak_config( fastrak_t *f, char c ) {
    int r = fprintf(f->fp, "%c\n", c);
    if( r != 2 )
    {
        fprintf(stderr, "Couldn't write fastrak config to %c, %d\n",
                c, r);
        perror("Fastrak error on write()");
        return -1;
    }
    return 0;
}

int fastrak_init( fastrak_t *f, const char *dev ) {
    f->dev = strdup(dev);
    int r;

    // open serial port
    f->tty_fd = open(f->dev, O_RDWR );
    if( -1 == f->tty_fd ) {
        fprintf(stderr, "Couldn't open fastrak device %s\n", f->dev);
        perror("Fastrak error on open()");
        return -1;
    }

    // set serial port
    tcgetattr(f->tty_fd, &f->tio); // Load the current setting
    f->tio.c_iflag=0;
    f->tio.c_oflag=0;
    f->tio.c_cflag=CS8|CREAD|CLOCAL; // set as per FASTRAK manual
    f->tio.c_lflag=0;
    f->tio.c_cc[VMIN]=47;
    f->tio.c_cc[VTIME]=5;
    cfsetospeed(&f->tio, B115200); // Set the output baud rates
    cfsetispeed(&f->tio, B115200); // Set the output baud rates
    f->tio.c_cflag |= (CLOCAL | CREAD); // Enable receiver, set local mode
    f->tio.c_cflag &= ~(unsigned)PARENB; // Parity bit
    f->tio.c_cflag &= ~(unsigned)CSTOPB; // Stop bits
    f->tio.c_cflag &= ~(unsigned)CSIZE;	 // Bit mask for data bits
    f->tio.c_cflag |= CS8;     // 8 data bit
    tcsetattr(f->tty_fd, TCSAFLUSH, &f->tio); // Set the new options for the port

    f->fp = fdopen( f->tty_fd, "r+");
    if( NULL == f->fp ) {
        fprintf(stderr, "Couldn't fdopen tty\n");
        perror("Fastrak error on fdopen()");
    }

    // set device params
    if( 0 != fastrak_config( f, 'P' ) ) {
        return -1;
    }

    for( size_t i = 0; i < 4; i++ ) {
        r = fprintf( f->fp, "O%c,52,61,50\n", i+48+1 );
        if( 12 != r ) {
            fprintf(stderr, "Couldn't write fastrak format [%d]: %d\n",
                    i, r);
            perror("fastrak error on write()");
            return -1;
        }
    }

    if( 0 != fastrak_config( f, 'u' )
        || 0 != fastrak_config( f, 'f' )
        || 0 != fastrak_config( f, 'C' )
        ) {
        return -1;
    }

    // flush
    r = fflush(f->fp);
    if( 0 != r ) {
        fprintf(stderr, "Couldn't flush tty: %d\n", r);
        perror( "fastrak error on fflush()");
    }

    return 0;

}

int fastrak_destroy( fastrak_t *f ) {
    free( f->dev );
    fclose( f->fp );
    return close(f->tty_fd);
}


int fastrak_read( fastrak_t *f ) {
    size_t j;
    int jc;
    int r;

    // cycle till next frame starts
    while( '0' != (r = getc(f->fp)) ) {
        if( EOF == r ) return -1;
    }

    // validate frame start
    r = getc(f->fp);
    if( EOF == r ) return -1;
    if( ! ('1' == r || '2' == r || '3' == r || '4' == r) )
        return 1;
    jc = r;
    r = getc(f->fp);
    if( EOF == r ) return -1;
    if( ' ' != r ) return 1;

    //read data
    uint8_t buf[7*4];
    size_t s = fread( buf, 4, 7, f->fp );
    if( s != 7 ) return -1;

    // validate frame end
    r = getc(f->fp);
    if( EOF == r ) return -1;
    if( ' ' != r ) return 1;

    // fill data
    assert( jc >= 49 );
    j = (size_t) (jc - 49); // jc - '1'
    assert( j <= 4 );
    for( size_t i = 0; i < 7; i++ ) {
        f->tf_vq[j][i] = aa_endconv_ld_le_s( buf + 4*i );
    }
    aa_la_scal( 3, 1.0/100, f->tf_vq[j] ); // convert cm to meter
    return 0;
}
