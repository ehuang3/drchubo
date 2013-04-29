#pragma once
#include "ZmpCommand.h"
#include "ZmpGains.h"

namespace zmp {

class ZmpDriver {
public:
	ZmpDriver();

	void loop();
	void next_command(ZmpCommand& _cmd, gain_list_t& _gains);

	void (*zmp_walker)(const ZmpCommand& _cmd, const gain_list_t& _gains);
	void (*zmp_reset)();

private:
	ZmpCommand zcmd;
	gain_list_t zgains;

	void driver_usage(std::ostream& _out);

	std::istream& get_code(std::vector<std::string> &code, std::string &line);

	void exec_driver(std::vector<std::string> &code, std::string &line);
	void do_cmd(std::vector<std::string> &code, std::string &line);
	void do_gains(std::vector<std::string> &code, std::string &line);
	void do_help(std::vector<std::string> &code, std::string &line);

	int best_match(std::string word, char cmds[][CMD_LEN], int cmdlen);
	int match(std::string word, char *cmd);
};

}
