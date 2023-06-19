#include <esc.h>
#include <iostream>

void esc::read_data_MPU6050() {
    while(true) {
      
        MPU_6050_Data mpu_data;
        mpu_data.accelerometerX = 3.0;
        mpu_data.accelerometerY = 3.0;
        mpu_data.accelerometerZ = 3.0;

        mpu_data.gyroscopeX = 5.0;
        mpu_data.gyroscopeY = 5.0;
        mpu_data.gyroscopeZ = 5.0;

        read_data_MPU6050.notify();
        f1.write(MPU_6050);
        cout << " data of MPU_6050 writing in FIFO ";
        std::cout << sc_time_stamp() << " :data of MPU read";
        wait(2, SC_SEC);
        read_data_MPU6050.cancel();
    }
}
void esc::read_data_VL53L1() {
    while(true) {
        //VL53L1->read();
        VL53L1_Data vl_data;
        vl_data.distance = 10.0;

        read_data_VL53L1.notify();
        cout << " data of VL53L1 writing in FIFO ";
        std::cout << sc_time_stamp() << " :data of VL53L1 read";


        fifo_2.write(mpu_data);
        wait(2, SC_SEC);
        read_data_VL53L1.cancel();
    }
}

void esc::esc_control() {
    while(true) {
        
        VL53L1_Data vl53_data;
        vl53_data.distance = 2.0;
        cout << " esc reading data !!!!"
        esc_control.notify();
        f1.read(MPU_6050_Data);
        f2.read(VL53L1_Data);

        safety_index = ((vl_data.distance*vl_data.distance)/(mpu_data.accelerometerX+mpu_data.accelerometerY-mpu_data.accelerometerZ))
        +((vl_data.distance*vl_data.distance)/(mpu_data.gyroscopeX+mpu_data.gyroscopeY-mpu_data.gyroscopeZ));
        if( safety_index > .. )
        {
            increament_1 = true;
            increament_2 = true;
            cout << " increa rate of propeller_1 and propeller_2 ";
        }
        if( safety_index < .... )
        {
            increament_3 = true;
            increament_4 = true;
            cout<< "increa rate of propeller_3 and propeller_4"
        }
        wait(2, SC_SEC);
        esc_control.cancel();
    }
}
SC_MODULE(testbench) {
    sc_port<sc_signal_out_if<bool>> decreament_1;
    sc_port<sc_signal_out_if<bool>> decreament_2;
    sc_port<sc_signal_out_if<bool>> decreament_3;
    sc_port<sc_signal_out_if<bool>> decreament_4;
    sc_port<sc_signal_out_if<bool>> increament_1;
    sc_port<sc_signal_out_if<bool>> increament_2;
    sc_port<sc_signal_out_if<bool>> increament_3;
    sc_port<sc_signal_out_if<bool>> increament_4;

    MPU_6050_Data mpu_data;
    VL53L1_Data vl_data;

    SC_CTOR(testbench) {
        SC_THREAD(simulation);
    }

    void simulation() {
        // valid Epass
        //cout << "Case 1: Valid Epass" << endl;
        decreament_1->write(0);
        decreament_2->write(0);
        decreament_3->write(0);
        decreament_4->write(0);
        increament_1->write(0);
        increament_2->write(0);
        increament_3->write(0);
        increament_4->write(0);

        wait(1, SC_SEC);
        mpu_data.accelerometerX = 3.0;
        mpu_data.accelerometerY = 3.0;
        mpu_data.accelerometerZ = 3.0;
        mpu_data.gyroscopeX = 5.0;
        mpu_data.gyroscopeY = 5.0;
        mpu_data.gyroscopeZ = 5.0;
        vl_data.distance = 10.0;
       
        wait(2, SC_SEC);

        mpu_data.accelerometerX = 4.0;
        mpu_data.accelerometerY = 4.0;
        mpu_data.accelerometerZ = 4.0;
        mpu_data.gyroscopeX = 6.0;
        mpu_data.gyroscopeY = 7.0;
        mpu_data.gyroscopeZ = 8.0;
        vl_data.distance = 9.0;
        
        wait( 3, SC_SEC);

       // valid Epass
        mpu_data.accelerometerX = 5.0;
        mpu_data.accelerometerY = 5.0;
        mpu_data.accelerometerZ = 7.0;
        mpu_data.gyroscopeX = 7.0;
        mpu_data.gyroscopeY = 9.0;
        mpu_data.gyroscopeZ = 4.0;
        vl_data.distance = 6.0;
        
        wait(4, SC_SEC);

        mpu_data.accelerometerX = 6.0;
        mpu_data.accelerometerY = 3.0;
        mpu_data.accelerometerZ = 8.0;
        mpu_data.gyroscopeX = 7.0;
        mpu_data.gyroscopeY = 4.0;
        mpu_data.gyroscopeZ = 9.0;
        vl_data.distance = 9.0;

        wait(4, SC_SEC);

        mpu_data.accelerometerX = 3.0;
        mpu_data.accelerometerY = 4.0;
        mpu_data.accelerometerZ = 6.0;
        mpu_data.gyroscopeX = 9.0;
        mpu_data.gyroscopeY = 5.0;
        mpu_data.gyroscopeZ = 6.0;
        vl_data.distance = 10.0;

        cout << "END!";
    }

};
int sc_main(int argc, char* argv[]) {
     
    testbench TEST("TEST");
    etc ESC("ESC");
    
    sc_signal<<bool> decreament_1;
    sc_signal<<bool> decreament_2;
    sc_signal<<bool> decreament_3;
    sc_signal<<bool> decreament_4;
   
    sc_signal<bool> increament_1;
    sc_signal<bool> increament_2;
    sc_signal<bool> increament_3;
    sc_signal<bool> increament_4;

    ESC.decreament_1(decreament_1);
    ESC.decreament_2(decreament_2);
    ESC.decreament_3(decreament_3);
    ESC.decreament_4(decreament_4);

    ESC.increament_1( increament_1);
    ESC.increament_2( increament_2);
    ESC.increament_3( increament_3);
    ESC.increament_4( increament_4);
     
     

     sc_start(50 , SC_SEC);
     return 0;
     return 0;
     }
