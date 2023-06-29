#include <systemc>
using namespace sc_core;
SC_MODULE(FIFO) {
  sc_fifo<double> f1;//, f2, f3;
   sc_in<bool> clk;


  SC_CTOR(FIFO) : f1(5) {
    SC_CTHREAD(read_data_MPU6050, clk.pos())
    SC_CTHREAD(read_data_VL53L1, clk);
    SC_CTHREAD(controler, clk.neg());
  }
 
void read_data_MPU6050() {
 while (true) {
    wait();
    //main_event.notify(i, SC_SEC);
    
    double  accelerometerX = 3.0;
    double accelerometerY = 4.0 ;
    double accelerometerZ = 5.0;
    double gyroscopeX = 3.0;
    double gyroscopeY = 5.0;
    double gyroscopeZ = 7.0;

    
   
      f1.write(accelerometerX);
      f1.write(accelerometerY);
      f1.write(accelerometerZ);
      f1.write(gyroscopeX);
      f1.write(gyroscopeY);
      f1.write(gyroscopeZ);
      std::cout << sc_time_stamp() << ":  read_data_MPU6050 accelerometerX writes " << accelerometerX++ << std::endl;
      std::cout << sc_time_stamp() << ":  read_data_MPU6050 accelerometerY writes " << accelerometerY++ << std::endl;
      std::cout << sc_time_stamp() << ": read_data_MPU6050 accelerometerZ writes " << accelerometerZ++ << std::endl;
     accelerometerX++;
     accelerometerY++;
     accelerometerZ++;
     gyroscopeX++;
     gyroscopeY++;
     gyroscopeZ++;
     
     
    }
  }
void read_data_VL53L1() {
  while(true)
  {
    wait( );
    //main_event.notify(i, SC_SEC);
    double distance = 6.0 ; 
   // double safety_index;
  f1.write(distance);
      // while (f1.nb_write(distance) == false ) {

      //   wait(f1.data_read_event());
      // }
      std::cout << sc_time_stamp() << ": read_data_VL53L1 writes " << distance << std::endl;
      
    }
  }
  void controler() {
    while(true)
    {
    wait();
    //main_event.notify(i, SC_SEC);
     double  accelerometerX ;
    double accelerometerY;
    double accelerometerZ;
    double gyroscopeX;
    double gyroscopeY;
    double gyroscopeZ;
    double distance;
    double safety_index;

      f1.read(accelerometerX);
      f1.read(accelerometerY);
      f1.read(accelerometerZ);
      f1.read(gyroscopeX);
      f1.read(gyroscopeY);
      f1.read(gyroscopeZ);
      f1.read(distance);
      std::cout << sc_time_stamp() << ": controler get " << accelerometerX << std::endl;
      std::cout << sc_time_stamp() << ": controler get " << accelerometerY << std::endl;
        std::cout << sc_time_stamp() << ": controler get " << accelerometerZ << std::endl;
        std::cout << sc_time_stamp() << ": controler get " << distance << std::endl;
         safety_index = (distance*distance)/(accelerometerX+accelerometerY-accelerometerZ)
        +((distance*distance)/(gyroscopeX+gyroscopeY-gyroscopeZ)); 
        if( safety_index > 30  )
    
        {
            std::cout << " ESC start " << " Decreatement thurst_1/2"<< std::endl;
        }
        else if( safety_index > 25 && safety_index < 30 )
        {
            std::cout << " ESC start " << " Decreatement thurst_3/4"<< std::endl;
        }
        else 
        {
            std::cout << " ESC NOT TURN ON " << std::endl;
        }
        
    
    }
  }
  
};
int sc_main(int, char*[]) {
 sc_clock clk("clk", 2, SC_SEC);
  FIFO fifo("fifo");
  fifo.clk(clk);
  sc_start(100, SC_SEC);
  return 0;
}