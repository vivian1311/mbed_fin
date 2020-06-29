#include "mbed.h"
#include "bbcar.h"

Serial pc(USBTX, USBRX);
DigitalOut blueLED(LED3);
// ping data
DigitalInOut pin10(D10);
parallax_ping ping(pin10);
// encoder
Ticker encoder_ticker;
DigitalIn pin3(D3);
parallax_encoder encoder_left(pin3, encoder_ticker);
// servo
Ticker servo_ticker;
PwmOut pin8(D8), pin9(D9);
BBCar car(pin8, pin9, servo_ticker);
// uart
Serial uart(D1, D0);
// xBee
RawSerial xbee(D12, D11);
EventQueue queue_xbee(32 * EVENTS_EVENT_SIZE);
Thread t_xbee;
// logger
Thread t_logger;


// function
void logger();
void data_matrix();
void get_image_data();
void object_detect();
void xbee_rx_interrupt(void);
void xbee_rx(void);

// global var
int obj_result = 0;  // for mission 2 
bool non_mission = 1;  // walking
int counter = 0;  // time counter
int movement = 0;  // 0: don't move; 1: straight; 2: back; 3: left; 4: right


int main() {
    pc.baud(9600);
    xbee.baud(9600);
    uart.baud(9600);

	pc.printf("start\r\n");
    blueLED = 1;
	t_xbee.start(callback(&queue_xbee, &EventQueue::dispatch_forever));
    xbee.attach(xbee_rx_interrupt, Serial::RxIrq);
    t_logger.start(logger);

    // walk to mission 1
    // straight
    wait(1);
    encoder_left.reset();
    movement = 1;
    car.goStraight(100);
    /*while (1){
        if ((float)ping < 35){
            car.stop();
            break;
        }
        wait_ms(10);
    }*/
    while(encoder_left.get_cm()<80) wait_ms(50);
    car.stop();

    // matrix data
    non_mission = 0;
    blueLED = 0;
    data_matrix();
    non_mission = 1;
    blueLED = 1;

    // turn left
    wait(1);
    encoder_left.reset();
    movement = 3;
    car.turn(100, 0.3);
    while(encoder_left.get_cm()<22) wait_ms(50);
    car.stop();
    
    // mission 1: 
    // straight
    wait(1);
    encoder_left.reset();
    movement = 1;
    car.goStraight(100);
    /*while (1){
        if ((float)ping < 35){
            car.stop();
            break;
        }
        wait_ms(10);
    }*/
    while(encoder_left.get_cm()<53) wait_ms(50);
    car.stop();
    // reverse parking
    wait(1);
    encoder_left.reset();
    movement = 3;
    car.turn(-100, 0.3);
    while(encoder_left.get_cm()<26) wait_ms(50);
    car.stop();
    // parking
    wait(1);
    encoder_left.reset();
    movement = 2;
    car.goStraight(-100);
    /*while (1){
        if ((float)ping > 55){
            car.stop();
            break;
        }
        wait_ms(10);
    }*/
    while(encoder_left.get_cm()<15) wait_ms(50);
    car.stop();
    // forward to pic
    /*wait(2);
    encoder_left.reset();
    movement = 1;
    car.goStraight(100);*/
    /*while (1){
        if ((float)ping < 35){
            car.stop();
            break;
        }
        wait_ms(10);
    }*/
    while(encoder_left.get_cm()<10) wait_ms(50);
    car.stop();


    wait(1);
    encoder_left.reset();
    movement = 4;
    car.turn(100, -0.3);
    while(encoder_left.get_cm()<8) wait_ms(50);
    car.stop();
    // take pic
    non_mission = 0;
    blueLED = 0;
    get_image_data();
    non_mission = 1;
    blueLED = 1;

    // walk to mission 2
    // turn right
    wait(1);
    encoder_left.reset();
    movement = 4;
    car.turn(100, -0.3);
    while(encoder_left.get_cm()<8) wait_ms(50);
    car.stop();
    // straight
    wait(1);
    encoder_left.reset();
    movement = 1;
    car.goStraight(100);
    /*while (1){
        if ((float)ping < 50){
            car.stop();
            break;
        }
        wait_ms(10);
    }*/
    while(encoder_left.get_cm()<20) wait_ms(50);
    car.stop();
    // turn right
    wait(1);
    encoder_left.reset();
    movement = 4;
    car.turn(100, -0.3);
    while(encoder_left.get_cm()<16) wait_ms(50);
    car.stop();
    // straight
    wait(1);
    encoder_left.reset();
    movement = 1;
    car.goStraight(100);
    /*while (1){
        if ((float)ping < 35){
            car.stop();
            break;
        }
        wait_ms(10);
    }*/
    while(encoder_left.get_cm()<67) wait_ms(50);
    car.stop();
    // turn right
    wait(1);
    encoder_left.reset();
    movement = 4;
    car.turn(100, -0.3);
    while(encoder_left.get_cm()<16) wait_ms(50);
    car.stop();

    // mission 2:
    // straight
    wait(1);
    encoder_left.reset();
    movement = 1;
    car.goStraight(100);
    /*while (1){
        if ((float)ping < 40){
            car.stop();
            break;
        }
        wait_ms(10);
    }*/
    while(encoder_left.get_cm()<23) wait_ms(50);
    car.stop();
    // turn right
    wait(1);
    encoder_left.reset();
    movement = 4;
    car.turn(100, -0.3);
    while(encoder_left.get_cm()<16) wait_ms(50);
    car.stop();
    // straight
    wait(1);
    encoder_left.reset();
    movement = 1;
    car.goStraight(100);
    /*while (1){
        if ((float)ping < 50){
            car.stop();
            break;
        }
        wait_ms(10);
    }*/
    while(encoder_left.get_cm()<8) wait_ms(50);
    car.stop();

    // object detect
    non_mission = 0;
    blueLED = 0;
    object_detect();
    // matrix data
    data_matrix();
    non_mission = 1;
    blueLED = 1;

    // walk to the end
    // back
    wait(1);
    encoder_left.reset();
    movement = 2;
    car.goStraight(-100);
    /*while (1){
        if ((float)ping > 30){
            car.stop();
            break;
        }
        wait_ms(10);
    }*/
    while(encoder_left.get_cm()<10) wait_ms(50);
    car.stop();
    // back right
    wait(1);
    encoder_left.reset();
    movement = 4;
    car.turn(-100, -0.3);
    while(encoder_left.get_cm()<16) wait_ms(50);
    car.stop();
    // straight
    wait(1);
    encoder_left.reset();
    movement = 1;
    car.goStraight(100);
    /*while (1){
        if ((float)ping < 35){
            car.stop();
            break;
        }
        wait_ms(10);
    }*/
    while(encoder_left.get_cm()<20) wait_ms(50);
    car.stop();
    // turn right
    wait(1);
    encoder_left.reset();
    movement = 4;
    car.turn(100, -0.3);
    while(encoder_left.get_cm()<16) wait_ms(50);
    car.stop();
    // straight
    wait(1);
    encoder_left.reset();
    movement = 1;
    car.goStraight(100);
    while(encoder_left.get_cm()<120) wait_ms(50);
    car.stop();

}

void logger(){
    while (1){
        if (non_mission){
            xbee.printf("%d %d\r\n", counter, movement);
            counter++;
            wait(1);
        }else{
            wait(1);
        }
    }
}

void data_matrix(){
    char s[10];
    sprintf(s, "matrix");
    uart.puts(s);
    wait(5);
    if (uart.readable())
    {
        char temp;
        char rev[20];
        int counter = 1;
        while (1){
            temp = uart.getc();
            if (temp == '\0'){
                temp = uart.getc();
            }
            else if (temp != '\r'){
                rev[counter] = temp;
                counter++;
            }
            else{
                break;
            }
        }
        for (int i = 1; i < counter; i++){
            xbee.putc(rev[i]);
        }
        counter = 1;
        xbee.printf("\r\n");
    }
}
void object_detect(){
    float data[3];
    int result = 1;
    // get data
    wait(0.5);
    data[0] = (float)ping;

    // right data
    car.turn(50, -0.05);
    wait(0.5);
    car.stop();
    wait(0.5);
    data[1] = (float)ping;

    // turn front
    car.turn(-50, -0.05);
    wait(0.5);
    car.stop();

    // left data
    car.turn(50, 0.05);
    wait(0.5);
    car.stop();
    wait(2);
    data[2] = (float)ping;

    // turn front
    car.turn(-50, 0.05);
    wait(0.5);
    car.stop();

    // calculate
    if (data[2] - data[0] > 3 && data[0] - data[1] > 3){
        obj_result = 2; // slope
    }else if (data[2] - data[0] > 3 && data[1] - data[0] > 3){
        obj_result = 3; // concave
    }else if (data[0] - data[2] > 3 && data[0] - data[1] > 3){
        obj_result = 0; // triangle
    }else{
        obj_result = 1; // square
    }
    // output result to xbee
    if (result == 0)
        xbee.printf("square\r\n");
    if (result == 1)
        xbee.printf("triangle\r\n");
    if (result == 2)
        xbee.printf("slope\r\n");
    if (result == 3)
        xbee.printf("concave\r\n");
    
}

void get_image_data(){
    // send quest
    char s[11];
    sprintf(s, "identify");
    uart.puts(s);
    wait(5);
    // fetch information
    if (uart.readable()){
        char recv = uart.getc();
        xbee.putc(recv);
        xbee.printf("get image\r\n");
    }
    
    
}

void xbee_rx_interrupt(void)
{
	xbee.attach(NULL, Serial::RxIrq); // detach interrupt
	queue_xbee.call(&xbee_rx);
}

void xbee_rx(void)
{
	char buf[100] = {0};
	char outbuf[100] = {0};
	while (xbee.readable())
	{
		for (int i = 0;; i++)
		{
			char recv = xbee.getc();
			if (recv == '\r')
			{
				break;
			}
			buf[i] = pc.putc(recv);
		}
		
		pc.printf("%s\r\n", outbuf);
		wait(0.1);
	}
	xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
}
