#! /bin/bash 
/tools/mentor/questa_2019.2_1/linux_x86_64/bin/qverifypm --monitor --host cadpc28 --port 44717 --wd /homes/user/stud/fall23/yl5334/tutorials_6863/FIFO --type master --binary /tools/mentor/questa_2019.2_1/linux_x86_64/bin/qverifyfk --id 0 -gui -tool prove -import_db Output_Results/formal_compile.db -od Output_Results -client_host cadpc28 -client_port 44489 -netcache /homes/user/stud/fall23/yl5334/tutorials_6863/FIFO/Output_Results/qcache/FORMAL/NET -pm_host cadpc28 -pm_port 44717   
