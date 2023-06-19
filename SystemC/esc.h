#include <systemc.h>
#include <iostream>

#define height 1 
#define g 9.8

struct MPU_6050_Data 
{
    double accelerometerX;
    double accelerometerY;
    double accelerometerZ;
    double gyroscopeX;
    double gyroscopeY;
    double gyroscopeZ;
};
struct VL53L1_Data 
{
    double distance;  
};
SC_MODULE( esc )
{	
    sc_mutex _mutex;
    sc_fifo<MPU_6050_Data> fifo_1; // fifo 1
    sc_fifo<VL53L1_Data> fifo_2;  // fifo 2

    sc_port<sc_signal_in_if<bool>> MPU_6050; 
    sc_port<sc_signal_in_if<bool>> VL53L1;
    
    static double safety_index;
    sc_in<MPU_6050_Data> MPU6050_Data_in;
    sc_in<VL53L1_Data> VL53L1_Data_in;

    sc_in<double> thurst_1;
    sc_in<double> thurst_2;
    sc_in<double> thurst_3;
    sc_in<double> thurst_4;

    sc_in<bool> decreament_1;
    sc_in<bool> decreament_1;
    sc_in<bool> decreament_1;
    sc_in<bool> decreament_1;

    sc_in<bool> increament_1;
    sc_in<bool> increament_2;
    sc_in<bool> increament_3;
    sc_in<bool> increament_4;

    sc_event read_data_MP6050;
    sc_event read_data_VL53L1;
    sc_event esc_control;
    sc_event read_data_VL53L1_pos();
    sc_event read_data_MPU6050_pos();

    SC_CTOR( esc ) : fifo_1(5) , fifo_2(5)
    {
        SC_THREAD(read_data_MP6050);
        SC_THREAD(read_data_VL53L1);
        SC_THREAD(esc_control);
    }
    void read_data_MPU6050();
    void read_data_VL53L1();
    void esc_control();
    void check_MPU_pos();
    void check_VL53L1_pos();
    };

	    



     
     
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
