#pragma once
#include <iostream>

namespace zmp {

enum walktype {
	walk_canned,
	walk_line,
	walk_circle
};

enum ik_error_sensitivity {
	ik_strict,
	ik_sloppy,
	ik_swing_permissive
};

struct ZmpCommand {
	bool show_gui = false;
	bool use_ros = false;
	walktype walk_type = walk_canned;
	double walk_circle_radius = 5.0;
	double walk_dist = 20;
	double footsep_y = 0.085; // half of horizontal separation distance between feet
	double foot_liftoff_z = 0.05; // foot liftoff height
	double step_length = 0.05;
	bool walk_sideways = false;
	double com_height = 0.48; // height of COM above ANKLE
	double com_ik_ascl = 0;
	double zmpoff_y = 0; // lateral displacement between zmp and ankle
	double zmpoff_x = 0;
	double lookahead_time = 2.5;
	double startup_time = 1.0;
	double shutdown_time = 1.0;
	double double_support_time = 0.05;
	double single_support_time = 0.70;
	size_t max_step_count = 4;
	double zmp_jerk_penalty = 1e-8; // jerk penalty on ZMP controller
	ik_error_sensitivity ik_sense = ik_strict;

	void fill(char *fpath);
	void fill(int argc, char** argv);
	void usage(std::ostream& ostr);
	friend std::ostream& operator<<(std::ostream& out, ZmpCommand& Z);
	void print_command(std::ostream& out);
};

class ZmpDriver {
public:
	void loop();
	ZmpCommand next_command();

	void (*zmp_walker)(const ZmpCommand& zcmd);
private:
	ZmpCommand zcmd;

	void driver_usage();

	void do_cmd(char **code, int len);
	void do_print(char **code, int len);
	void do_run(char **code, int len);
	void do_fill(char **code, int len);

	int best_match(char *word, char **cmds, int cmd_size);
	int match(char *word, char *cmd);
};

}
