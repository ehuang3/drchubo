#pragma once
#include "ZmpCommand.h"

namespace zmp {

class ZmpDriver {
public:
	ZmpDriver();

	void loop();
	ZmpCommand next_command();

	void (*zmp_walker)(const ZmpCommand& zcmd);
private:
	ZmpCommand zcmd;

	void driver_usage(std::ostream& out);

	std::istream& get_code(std::vector<std::string> &code);

	void do_cmd(std::vector<std::string> code);
	void do_print(std::vector<std::string> code);
	void do_run(std::vector<std::string > code);
	void do_fill(std::vector<std::string> code);
	void do_help(std::vector<std::string> code);

	int best_match(std::string word, char cmds[][CMD_LEN], int cmd_size);
	int match(std::string word, char *cmd);
};

}
