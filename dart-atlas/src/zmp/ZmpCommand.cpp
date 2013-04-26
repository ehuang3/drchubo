#include "ZmpCommand.h"
#include <stdio.h>
#include <getopt.h>

namespace zmp {

static void _usage(std::ostream& ostr) {
	ostr <<
			"usage: zmp [OPTIONS]\n"
			"\n"
			"OPTIONS:\n"
			"\n"
			"  -g, --show-gui                    Show a GUI after computing trajectories.\n"
			"  -R, --use-ros                     Send trajectory via ROS after computing.\n"
			"  -I, --ik-errors                   IK error handling: strict/sloppy\n"
			"  -w, --walk-type                   Set type: canned/line/circle\n"
			"  -D, --walk-distance               Set maximum distance to walk\n"
			"  -r, --walk-circle-radius          Set radius for circle walking\n"
			"  -c, --max-step-count=NUMBER       Set maximum number of steps\n"
			"  -y, --foot-separation-y=NUMBER    Half-distance between feet\n"
			"  -z, --foot-liftoff-z=NUMBER       Vertical liftoff distance of swing foot\n"
			"  -l, --step-length=NUMBER          Max length of footstep\n"
			"  -S, --walk-sideways               Should we walk sideways? (canned gait only)\n"
			"  -h, --com-height=NUMBER           Height of the center of mass\n"
			"  -a, --comik-angle-weight=NUMBER   Angle weight for COM IK\n"
			"  -Y, --zmp-offset-y=NUMBER         Lateral distance from ankle to ZMP\n"
			"  -X, --zmp-offset-x=NUMBER         Forward distance from ankle to ZMP\n"
			"  -T, --lookahead-time=NUMBER       Lookahead window for ZMP preview controller\n"
			"  -p, --startup-time=NUMBER         Initial time spent with ZMP stationary\n"
			"  -n, --shutdown-time=NUMBER        Final time spent with ZMP stationary\n"
			"  -d, --double-support-time=NUMBER  Double support time\n"
			"  -s, --single-support-time=NUMBER  Single support time\n"
			"  -P, --zmp-jerk-penalty=NUMBER     P-value for ZMP preview controller\n"
			"  -H, --help                        See this message\n";
}

static double getdouble(const char* str) {
	char* endptr;
	double d = strtod(str, &endptr);
	if (!endptr || *endptr) {
		std::cerr << "Error parsing number on command line!\n\n";
//		_usage(std::cerr);
//		exit(1);
		return 0;
	}
	return d;
}

static long getlong(const char* str) {
	char* endptr;
	long d = strtol(str, &endptr, 10);
	if (!endptr || *endptr) {
		std::cerr << "Error parsing number on command line!\n\n";
//		_usage(std::cerr);
//		exit(1);
		return 0;
	}
	return d;
}

static ik_error_sensitivity getiksense(const std::string& s) {
	if (s == "strict") {
		return ik_strict;
	} else if (s == "sloppy") {
		return ik_sloppy;
	} else if (s == "permissive") {
		return ik_swing_permissive;
	} else {
		std::cerr << "bad ik error sensitivity " << s << "\n";
//		_usage(std::cerr);
//		exit(1);
		return 0;
	}
}

static walktype getwalktype(const std::string& s) {
	if (s == "canned") {
		return walk_canned;
	} else if (s == "line") {
		return walk_line;
	} else if (s == "circle") {
		return walk_circle;
	} else {
		std::cerr << "bad walk type " << s << "\n";
//		_usage(std::cerr);
//		exit(1);
		return 0;
	}
}

void ZmpCommand::fill(char *fpath) {

}

void ZmpCommand::fill(int argc, char** argv) {
	const struct option long_options[] = {
			{ "show-gui",            no_argument,       0, 'g' },
			{ "use-ros",             no_argument,       0, 'R' },
			{ "ik-errors",           required_argument, 0, 'I' },
			{ "walk-type",           required_argument, 0, 'w' },
			{ "walk-distance",       required_argument, 0, 'D' },
			{ "walk-circle-radius",  required_argument, 0, 'r' },
			{ "max-step-count",      required_argument, 0, 'c' },
			{ "foot-separation-y",   required_argument, 0, 'y' },
			{ "foot-liftoff-z",      required_argument, 0, 'z' },
			{ "step-length",         required_argument, 0, 'l' },
			{ "walk-sideways",       no_argument,       0, 'S' },
			{ "com-height",          required_argument, 0, 'h' },
			{ "comik-angle-weight",  required_argument, 0, 'a' },
			{ "zmp-offset-y",        required_argument, 0, 'Y' },
			{ "zmp-offset-x",        required_argument, 0, 'X' },
			{ "lookahead-time",      required_argument, 0, 'T' },
			{ "startup-time",        required_argument, 0, 'p' },
			{ "shutdown-time",       required_argument, 0, 'n' },
			{ "double-support-time", required_argument, 0, 'd' },
			{ "single-support-time", required_argument, 0, 's' },
			{ "zmp-jerk-penalty",    required_argument, 0, 'P' },
			{ "help",                no_argument,       0, 'H' },
			{ 0,                     0,                 0,  0  },
	};

	const char* short_options = "gRI:w:D:r:c:y:z:l:Sh:a:Y:X:T:p:n:d:s:P:H";

	int opt, option_index;

	while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 ) {
		switch (opt) {
		case 'g': show_gui = true; break;
		case 'R': use_ros = true; break;
		case 'I': ik_sense = getiksense(optarg); break;
		case 'w': walk_type = getwalktype(optarg); break;
		case 'D': walk_dist = getdouble(optarg); break;
		case 'r': walk_circle_radius = getdouble(optarg); break;
		case 'c': max_step_count = getlong(optarg); break;
		case 'y': footsep_y = getdouble(optarg); break;
		case 'z': foot_liftoff_z = getdouble(optarg); break;
		case 'l': step_length = getdouble(optarg); break;
		case 'S': walk_sideways = true; break;
		case 'h': com_height = getdouble(optarg); break;
		case 'a': com_ik_ascl = getdouble(optarg); break;
		case 'Y': zmpoff_y = getdouble(optarg); break;
		case 'X': zmpoff_x = getdouble(optarg); break;
		case 'T': lookahead_time = getdouble(optarg); break;
		case 'p': startup_time = getdouble(optarg); break;
		case 'n': shutdown_time = getdouble(optarg); break;
		case 'd': double_support_time = getdouble(optarg); break;
		case 's': single_support_time = getdouble(optarg); break;
		case 'P': zmp_jerk_penalty = getdouble(optarg); break;
		case 'H': usage(std::cout); break;
		default:  usage(std::cerr); break;
		}
	}
}

void ZmpCommand::usage(std::ostream& ostr) {
	_usage(ostr);
}

static char *walk2str(walktype walk) {
	char *str = "ERROR: Not set";
	if(walk == walk_canned) str = "walk_canned";
	if(walk == walk_line) str = "walk_line";
	if(walk == walk_circle) str = "walk_circle";
	return str;
}

static char *ik2str(ik_error_sensitivity ik_err) {
	char *str = "ERROR: Not set";
	if(ik_err == ik_strict) str = "ik_strict";
	if(ik_err == ik_sloppy) str = "ik_sloppy";
	if(ik_err == ik_swing_permissive) str = "ik_swing_permissive";
	return str;
}

std::ostream& operator<<(std::ostream& out, ZmpCommand& Z) {
	return
	out << "  -g, --show-gui                    " << Z.show_gui << "\n"
		<< "  -R, --use-ros                     " << Z.use_ros << "\n"
		<< "  -I, --ik-errors                   " << ik2str(Z.ik_sense) << "\n"
		<< "  -w, --walk-type                   " << walk2str(Z.walk_type) << "\n"
		<< "  -D, --walk-distance               " << Z.walk_dist << "\n"
		<< "  -r, --walk-circle-radius          " << Z.walk_circle_radius << "\n"
		<< "  -c, --max-step-count              " << Z.max_step_count << "\n"
		<< "  -y, --foot-separation-y           " << Z.footsep_y << "\n"
		<< "  -z, --foot-liftoff-z              " << Z.foot_liftoff_z << "\n"
		<< "  -l, --step-length                 " << Z.step_length << "\n"
		<< "  -S, --walk-sideways               " << Z.walk_sideways << "\n"
		<< "  -h, --com-height                  " << Z.com_height << "\n"
		<< "  -a, --comik-angle-weight          " << Z.com_ik_ascl << "\n"
		<< "  -Y, --zmp-offset-y                " << Z.zmpoff_y << "\n"
		<< "  -X, --zmp-offset-x                " << Z.zmpoff_x << "\n"
		<< "  -T, --lookahead-time              " << Z.lookahead_time << "\n"
		<< "  -p, --startup-time                " << Z.startup_time << "\n"
		<< "  -n, --shutdown-time               " << Z.shutdown_time << "\n"
		<< "  -d, --double-support-time         " << Z.double_support_time << "\n"
		<< "  -s, --single-support-time         " << Z.single_support_time << "\n"
		<< "  -P, --zmp-jerk-penalty            " << Z.zmp_jerk_penalty << "\n"
		<< "\n";
}

void ZmpCommand::print_command(std::ostream& out) {
	out << *this;
}

void ZmpDriver::do_cmd(char **code, int len) {
	char cmds[][] = { "print", "fill", "run" };
	switch (best_match(code[0], cmds, 3)) {
	case 0: do_print(code, len); break;
	case 1: do_fill(code, len); break;
	case 2: do_run(code, len); break;
	default: break;
	}
}

void ZmpDriver::do_print(char **code, int len) {
	char args[][] = { "cmd", "gains" };
}

void ZmpDriver::do_run(char **code, int len) {

}

void ZmpDriver::do_fill(char **code, int len) {

}

int ZmpDriver::best_match(char *word, char **cmds, int cmd_size) {
	int max=0;
	int best=-1;
	for(int i=0; i < cmd_size; i++) {
		int m = match(word, cmds[i]);
		if(m > max) {
			best = i;
			max = m;
		}
	}
	return cmds[best];
}

int ZmpDriver::match(char *word, char *cmd) {
	int i = 0;
	while(word[i] != 0 && cmd[i] != 0 && word[i] == cmd[i]) {
		i++;
	}
	return i;
}



}
