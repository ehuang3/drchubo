#pragma once
#include <iostream>
#include <vector>
#include <string>

namespace zmp {

const int CMD_LEN = 256;

enum walktype {
	walk_invalid,
	walk_canned,
	walk_line,
	walk_circle
};

enum ik_error_sensitivity {
	ik_invalid,
	ik_strict,
	ik_sloppy,
	ik_swing_permissive
};

struct ZmpCommand {
	bool show_gui;
	bool use_ros;
	walktype walk_type;
	double walk_circle_radius;
	double walk_dist;
	double footsep_y; // half of horizontal separation distance between feet
	double foot_liftoff_z; // foot liftoff height
	double step_length;
	bool walk_sideways;
	double com_height; // height of COM above ANKLE
	double com_ik_ascl;
	double zmpoff_y; // lateral displacement between zmp and ankle
	double zmpoff_x;
	double lookahead_time;
	double startup_time;
	double shutdown_time;
	double double_support_time;
	double single_support_time;
	size_t max_step_count;
	double zmp_jerk_penalty; // jerk penalty on ZMP controller
	ik_error_sensitivity ik_sense;

	ZmpCommand();

	void fill(const char *fpath);
	void fill(std::vector<std::string>::iterator words);
	void fill(std::string param, std::string val);
	void fill(int argc, char * const argv[]);
	void usage(std::ostream& ostr) const;
	friend std::ostream& operator<<(std::ostream& out, const ZmpCommand& Z);
	void print_command(std::ostream& out) const;
};

}
