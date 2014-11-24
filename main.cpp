#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>
#include <boost/program_options.hpp>
#include "ils.h"
#include "jit.h"
using namespace std;
using namespace boost::program_options;

int main(int argc, char *argv[]) {
  options_description options1("simulator control");

  options1.add_options()
      ("sim,s", value<string>()->default_value("ils"),
                "which implementation to use (ils,jit)")
      ("help,h", "show help")
  ;
  variables_map values;
  try {
    store(parse_command_line(argc, argv, options1), values);
    notify(values);
    string sim_impl = values["sim"].as<string>();
    if(values.count("help")) {
      cerr << options1 << endl;
    } else if(sim_impl == "ils") {
      ils_main();
    } else if(sim_impl == "jit") {
      jit_main();
    } else {
      cerr << "Unknown implementation name : " << sim_impl << endl;
      exit(1);
    }
  } catch(exception &e) {
    cerr << e.what() << endl;
    exit(1);
  }
  return 0;
}
