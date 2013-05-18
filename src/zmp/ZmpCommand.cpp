#include "ZmpCommand.h"
#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iterator>
#include <algorithm>
#define DEBUG
#define MODULE_NAME "ZMP-CMD"
#include "ZmpDebug.h"

using namespace std;
namespace zmp {

static void _usage(std::ostream& ostr) {
	ostr << "ZMP-CMD OPTIONS:\n"
			"  g, show-gui                    Show a GUI after computing trajectories.\n"
			"  R, use-ros                     Send trajectory via ROS after computing.\n"
			"  I, ik-errors                   IK error handling: strict/sloppy\n"
			"  w, walk-type                   Set type: canned/line/circle\n"
			"  D, walk-distance               Set maximum distance to walk\n"
			"  r, walk-circle-radius          Set radius for circle walking\n"
			"  c, max-step-count=NUMBER       Set maximum number of steps\n"
			"  y, foot-separation-y=NUMBER    Half-distance between feet\n"
			"  z, foot-liftoff-z=NUMBER       Vertical liftoff distance of swing foot\n"
			"  l, step-length=NUMBER          Max length of footstep\n"
			"  S, walk-sideways               Should we walk sideways? (canned gait only)\n"
			"  h, com-height=NUMBER           Height of the center of mass\n"
			"  a, comik-angle-weight=NUMBER   Angle weight for COM IK\n"
			"  Y, zmp-offset-y=NUMBER         Lateral distance from ankle to ZMP\n"
			"  X, zmp-offset-x=NUMBER         Forward distance from ankle to ZMP\n"
			"  T, lookahead-time=NUMBER       Lookahead window for ZMP preview controller\n"
			"  p, startup-time=NUMBER         Initial time spent with ZMP stationary\n"
			"  n, shutdown-time=NUMBER        Final time spent with ZMP stationary\n"
			"  d, double-support-time=NUMBER  Double support time\n"
			"  s, single-support-time=NUMBER  Single support time\n"
			"  P, zmp-jerk-penalty=NUMBER     P-value for ZMP preview controller\n";
}

static double getdouble(const char* str) {
	char* endptr;
	double d = strtod(str, &endptr);
	if (!endptr || *endptr) {
		ERROR_PRINT("Failed to parse number: %s\n", str);
		return -1;
	}
	return d;
}

static long getlong(const char* str) {
	char* endptr;
	long d = strtol(str, &endptr, 10);
	if (!endptr || *endptr) {
		ERROR_PRINT("Failed to parse number: %s\n", str);
		return -1;
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
		ERROR_STREAM << "Failed to parse ik-error-sensitivity: " << s << "\n";
		return ik_invalid;
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
		ERROR_STREAM << "Failed to parse walktype: " << s << "\n";
		return walk_invalid;
	}
}

ZmpCommand::ZmpCommand()
	: show_gui(false),
	  use_ros(false),
	  walk_type(walk_canned),
	  walk_circle_radius(5.0),
	  walk_dist(20),
	  footsep_y(0.085),
	  foot_liftoff_z(0.05),
	  step_length(0.05),
	  walk_sideways(false),
	  com_height(0.48),
	  com_ik_ascl(0),
	  zmpoff_y(0),
	  zmpoff_x(0),
	  lookahead_time(2.5),
	  startup_time(1.0),
	  shutdown_time(1.0),
	  double_support_time(0.05),
	  single_support_time(0.70),
	  max_step_count(4),
	  zmp_jerk_penalty(1e-8),
	  ik_sense(ik_strict)
{
}

void ZmpCommand::fill(const char *fpath) {
	ifstream ifs(fpath);
	if(!ifs) {
		// file could not be opened
		ERROR_PRINT("Failed to open file: %s\n", fpath);
		return;
	}
	// Grab line by line and pass to fill(argc,argv)
	string line;
	vector<string> code;
	while(getline(ifs, line)) {
		if(line.empty() && ifs.eof()) {
			//FIXME: if-statement doesn't work
			DEBUG_PRINT("End of input\n");
			continue;
		}
		// tokenize input
		istringstream iss(line);
		code.clear();
		copy(istream_iterator<string>(iss),
			 istream_iterator<string>(),
			 back_inserter<vector<string> >(code));
		if(code.size() == 0) {
			// empty line ...
			continue;
		}
		if(code.size() != 2) {
			ERROR_PRINT("Bad # of tokens (%ld): %s\n", code.size(), line.c_str());
			continue;
		}
		fill(code[0], code[1]);
	}
}

void ZmpCommand::fill(string param, string val) {
	DEBUG_PRINT("Writing into ZMP-CMD: %21s %s\n", param.c_str(), val.c_str());
	if(param[0] != '-') {
		if(param.size() > 1) {
			param = "-" + param;
		}
		param = "-" + param;
	}
	// fake argc, argv
	char * const argv[3] = { "zmp", (char*)param.c_str(), (char*)val.c_str() };
	fill(3, argv);
}

void ZmpCommand::fill(int argc, char * const argv[]) {
	const struct option long_options[] = {
			{ "show-gui",            required_argument, 0, 'g' },
			{ "use-ros",             required_argument, 0, 'R' },
			{ "ik-errors",           required_argument, 0, 'I' },
			{ "walk-type",           required_argument, 0, 'w' },
			{ "walk-distance",       required_argument, 0, 'D' },
			{ "walk-circle-radius",  required_argument, 0, 'r' },
			{ "max-step-count",      required_argument, 0, 'c' },
			{ "foot-separation-y",   required_argument, 0, 'y' },
			{ "foot-liftoff-z",      required_argument, 0, 'z' },
			{ "step-length",         required_argument, 0, 'l' },
			{ "walk-sideways",       required_argument, 0, 'S' },
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

	char opt;
	int option_index;

	// reset!!! wtf
	optind = 0;

	while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 ) {
		switch (opt) {
		case 'g': show_gui = getlong(optarg); break;
		case 'R': use_ros = getlong(optarg); break;
		case 'I': ik_sense = getiksense(optarg); break;
		case 'w': walk_type = getwalktype(optarg); break;
		case 'D': walk_dist = getdouble(optarg); break;
		case 'r': walk_circle_radius = getdouble(optarg); break;
		case 'c': max_step_count = getlong(optarg); break;
		case 'y': footsep_y = getdouble(optarg); break;
		case 'z': foot_liftoff_z = getdouble(optarg); break;
		case 'l': step_length = getdouble(optarg); break;
		case 'S': walk_sideways = getlong(optarg); break;
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
		default:
			ERROR_PRINT("Invalid parameters: %s %s\n", argv[1], argv[2]);
			break;
		}
	}
}

void ZmpCommand::usage(std::ostream& ostr) const {
	_usage(ostr);
}

static string walk2str(walktype walk) {
	string str = "ERROR: Not set";
	if(walk == walk_canned) str = "canned";
	if(walk == walk_line) str = "line";
	if(walk == walk_circle) str = "circle";
	return str;
}

static string ik2str(ik_error_sensitivity ik_err) {
	string str = "ERROR: Not set";
	if(ik_err == ik_strict) str = "strict";
	if(ik_err == ik_sloppy) str = "sloppy";
	if(ik_err == ik_swing_permissive) str = "permissive";
	return str;
}

std::ostream& operator<<(std::ostream& out, const ZmpCommand& Z) {
	return
	out << "show-gui                    " << Z.show_gui << "\n"
		<< "use-ros                     " << Z.use_ros << "\n"
		<< "ik-errors                   " << ik2str(Z.ik_sense) << "\n"
		<< "walk-type                   " << walk2str(Z.walk_type) << "\n"
		<< "walk-distance               " << Z.walk_dist << "\n"
		<< "walk-circle-radius          " << Z.walk_circle_radius << "\n"
		<< "max-step-count              " << Z.max_step_count << "\n"
		<< "foot-separation-y           " << Z.footsep_y << "\n"
		<< "foot-liftoff-z              " << Z.foot_liftoff_z << "\n"
		<< "step-length                 " << Z.step_length << "\n"
		<< "walk-sideways               " << Z.walk_sideways << "\n"
		<< "com-height                  " << Z.com_height << "\n"
		<< "comik-angle-weight          " << Z.com_ik_ascl << "\n"
		<< "zmp-offset-y                " << Z.zmpoff_y << "\n"
		<< "zmp-offset-x                " << Z.zmpoff_x << "\n"
		<< "lookahead-time              " << Z.lookahead_time << "\n"
		<< "startup-time                " << Z.startup_time << "\n"
		<< "shutdown-time               " << Z.shutdown_time << "\n"
		<< "double-support-time         " << Z.double_support_time << "\n"
		<< "single-support-time         " << Z.single_support_time << "\n"
		<< "zmp-jerk-penalty            " << Z.zmp_jerk_penalty << "\n"
		<< "\n";
}

void ZmpCommand::print_command(std::ostream& out) const {
	out << *this;
}

}
