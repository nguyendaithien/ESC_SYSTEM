Step Run program :
 +, # export SYSTEMC_HOME=/usr/local/systemc-2.3.1/
 +, # g++ -I. -I$SYSTEMC_HOME/include -L. -L$SYSTEMC_HOME/lib-linux64 -Wl,-rpath=$SYSTEMC_HOME/lib-linux64 -o esc esc.cpp -lsystemc -lm
 +, # ./esc
