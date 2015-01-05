#include <stdio.h>
#include <math.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>

#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <linux/joystick.h>

#include "tick.h"
#include <wiringPi.h>
#include "BrickPi.h"

// USBミサイルランチャ関連ヘッダ
#include <ctype.h>
#include <unistd.h>
#include <usb.h>
#include <opencv/highgui.h>


#define HID_REPORT_SET (0x09)
#define BULK_OUT (0x02)

#define MISSILE_VENDOR_ID (0x2123)
#define MISSILE_PRODUCT_ID (0x1010)
#define MISSILE_CMD_HEADER (0x02)
#define MISSILE_CMD_DOWN (0x01)
#define MISSILE_CMD_UP (0x02)
#define MISSILE_CMD_LEFT (0x04)
#define MISSILE_CMD_RIGHT (0x08)
#define MISSILE_CMD_FIRE (0x10)
#define MISSILE_CMD_STOP (0x20)

#define MAX_ANALOG_RANGE (32767)
#define MIN_ANALOG_RANGE (-32767)
#define ANALOG_RANGE (MAX_ANALOG_RANGE - MIN_ANALOG_RANGE)
#define MAX_MOTOR_SPEED (100)

#define WALK_CMD_STOP (0x00)
#define WALK_CMD_FORWARD (0x01)
#define WALK_CMD_BACK (0x02)
#define WALK_CMD_TURN_LEFT (0x04)
#define WALK_CMD_TURN_RIGHT (0x08)

const int USB_TIMEOUT = 3000;
const int BUF_SIZE = 8;

int main(int argc, char *argv[])
{
	int result;
	int motor[2];
	int sensor[2];
	int i, j;
	int speedL, speedR;
	int missileCmd;
	int walkCmd;
	int lastWalkCmd = (int)(WALK_CMD_STOP);
	int legInit = 0;

	// USB関連変数。
	struct usb_bus *bus;
	struct usb_device *dev;
	usb_dev_handle *udh;
	struct usb_config_descriptor *config;
	struct usb_interface *interface;
	struct usb_interface_descriptor *altsetting;
	unsigned char usbSendData[8];

	// 画像処理関連変数。
	CvCapture *capture = 0;
	IplImage *frame = 0;
	//double width = 160, height = 120;
	double width = 320, height = 240;

	// ジョイスティック変数。
	int *axis;
	char *button;
	int numOfAxis;
	int numOfButton;
	int fd_j;
	struct js_event js;
	
	// -- Joystick 初期化 --
	// デバイスオープン
	if ((fd_j = open("/dev/input/js0", O_RDONLY)) < 0)
	{
		printf("Error in Opening Joystick.\n");
	}
	// 軸・ボタン数取得。
	ioctl(fd_j, JSIOCGAXES, &numOfAxis);
	ioctl(fd_j, JSIOCGBUTTONS, &numOfButton);
	// 今回は軸数・ボタン数の取得がうまく行かなかったので、固定で。
	numOfAxis = 19;
	numOfButton =16;
	axis = (int*)calloc(numOfAxis, sizeof(int));
	for (i = 0; i < numOfAxis; i++)
		axis[i] = 0;
	button = (char*)calloc(numOfButton, sizeof(char));
	for (i = 0; i < numOfButton; i++)
		button[i] = 0;
	// ノンブロッキングモード。
	fcntl(fd_j, F_SETFL, O_NONBLOCK);


	// -- BrickPi 初期化 --
	// タイマクリア。
	ClearTick();
	// ポートオープン。
	result = BrickPiSetup();
	if (result)
	{
		printf("BrickPi Initializing Error.");
		return -1;
	}
	// アドレス設定。
	BrickPi.Address[0] = 1;
	BrickPi.Address[1] = 2;
	
	// モータポート設定。
	motor[0] = PORT_B;	// PORT_BをLモータとして使用。
	motor[1] = PORT_C;	// PORT_CをRモータとして使用。
	// モータ有効化。
	for (i = 0; i < 2; i++)
		BrickPi.MotorEnable[motor[i]] = 1;
	// タッチセンサ利用の設定。PORT_2,3 を使用。
	sensor[0] = PORT_2;
	sensor[1] = PORT_3;
	for (i = 0; i < 2; i++)
		BrickPi.SensorType[sensor[i]] = TYPE_SENSOR_TOUCH;
	// BrickPiの設定を更新。
	result = BrickPiSetupSensors();
	if (result)
	{
		printf("BrickPi Setup Error.");
		return -1;
	}

	// 左右脚の初期位置への移動。
	for (i = 0; i < 2; i++)
	{
		BrickPi.MotorSpeed[motor[i]] = 100;
		while (1)
		{
			BrickPiUpdateValues();
			usleep(10000);
			if (BrickPi.Sensor[sensor[i]] > 0)
			{
				BrickPi.MotorSpeed[motor[i]] = 0;
				BrickPi.Encoder[motor[i]] = 0;
				break;
			}
		}
	}
	

	// --- USBミサイルランチャ初期化 ---
	// USB初期化。
	usb_init();
	usb_find_busses();
	usb_find_devices();
	for (i = 0; i < 8; i++)
		usbSendData[i] = 0x00;
	usbSendData[0] = MISSILE_CMD_HEADER;
	
	// ミサイルランチャーのデバイスチェック。
	for(bus = usb_get_busses(); bus; bus = bus->next)
	{
		for(dev = bus->devices; dev; dev = dev->next)
		{
			if(dev->descriptor.idVendor == MISSILE_VENDOR_ID &&
				dev->descriptor.idProduct == MISSILE_PRODUCT_ID)
			{
				// デバイスオープン。
				if((udh = usb_open(dev)) == NULL)
				{
					// オープンに失敗。
					printf("usb_open Error.(%s)\n", usb_strerror());
					exit(-1);
				}
				
				printf("found usb missile launcher\n");

				config = &dev->config[0];
				interface = &config->interface[0];
				altsetting = &interface->altsetting[0];
				
				if (usb_set_configuration(udh, config->bConfigurationValue) < 0)
				{
					if (usb_detach_kernel_driver_np(udh,
														altsetting->bInterfaceNumber) < 0)
					{
						printf("usb_set_configration() error.\n");
						usb_close(udh);
						exit(-1);
					}
				}
				
				// インタフェースの使用をシステムに通知。
				if (usb_claim_interface(udh, altsetting->bInterfaceNumber) < 0)
				{
					// インタフェース使用要求でエラー。
					printf("claiming interface error\n");
					exit(-1);
				}
			}
		}
	}
	/*
	// カメラに対するキャプチャ構造体を作成。
	if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
		capture = cvCreateCameraCapture(argc == 2 ? argv[1][0] - '0' : 0);
	
	// キャプチャサイズの設定。
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, width);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, height);
	
	// ウィンドウ作成。
	cvNamedWindow("Capture", CV_WINDOW_AUTOSIZE);
	*/

	// --- メインループ ---
	while(1)
	{
		// Joystick読み出し。
		if (read(fd_j, &js, sizeof(struct js_event)) == sizeof(struct js_event))
		{
			switch (js.type & ~JS_EVENT_INIT)
			{
			case JS_EVENT_AXIS:		// スティック操作イベント
				axis[js.number] = js.value;
				break;
			case JS_EVENT_BUTTON:	// ボタン操作イベント
				button[js.number] = js.value;
				break;
			}
		}

		// --- Joystick値から動作情報生成 ---
		// axis[0]=左アナログX軸, axis[1]=左アナログY軸, axis[2]=右アナログX軸, axis[3]=右アナログY軸
		// 移動情報生成。
		walkCmd = WALK_CMD_STOP;
		if (axis[1] < (int)(MIN_ANALOG_RANGE /3))
		{
			walkCmd |= WALK_CMD_FORWARD;
		}
		else if (axis[1] > (int)(MAX_ANALOG_RANGE / 3))
		{	
			walkCmd |= WALK_CMD_BACK;
		}
		if (axis[0] < (int)(MIN_ANALOG_RANGE /3))
		{
			walkCmd |= WALK_CMD_TURN_LEFT;
		}
		else if (axis[0] > (int)(MAX_ANALOG_RANGE /3))
		{
			walkCmd |= WALK_CMD_TURN_RIGHT;
		}
		switch (walkCmd)
		{
		case WALK_CMD_FORWARD:
			BrickPi.MotorSpeed[motor[0]] = -200;
			BrickPi.MotorSpeed[motor[1]] = -200;
			break;
		case WALK_CMD_BACK:
			BrickPi.MotorSpeed[motor[0]] = 200;
			BrickPi.MotorSpeed[motor[1]] = 200;
			break;
		case WALK_CMD_TURN_LEFT:
			BrickPi.MotorSpeed[motor[0]] = -200;
			BrickPi.MotorSpeed[motor[1]] = 200;
			break;
		case WALK_CMD_TURN_RIGHT:
			BrickPi.MotorSpeed[motor[0]] = 200;
			BrickPi.MotorSpeed[motor[1]] = -200;
			break;
		case WALK_CMD_STOP:
			if ((legInit == 0) && (lastWalkCmd != WALK_CMD_STOP))	legInit = 1;
			BrickPi.MotorSpeed[motor[0]] = 0;
			BrickPi.MotorSpeed[motor[1]] = 0;
			break;
		default:
			break;
		}

		// ミサイルコマンド生成。
		missileCmd = 0x00;
		if (axis[2] < (int)(MIN_ANALOG_RANGE / 3))
		{
			missileCmd |= MISSILE_CMD_LEFT;
		}
		else if (axis[2] > (int)(MAX_ANALOG_RANGE / 3))
		{
			missileCmd |= MISSILE_CMD_RIGHT;
		}
		if (axis[3] < (int)(MIN_ANALOG_RANGE / 3))
		{
			missileCmd |= MISSILE_CMD_UP;
		}
		else if (axis[3] > (int)(MAX_ANALOG_RANGE / 3))
		{
			missileCmd |= MISSILE_CMD_DOWN;
		}
		if (axis[13] > (int)(MAX_ANALOG_RANGE / 3))
		{
			missileCmd |= MISSILE_CMD_FIRE;
		}
		// --- 動作処理 ---
		BrickPiUpdateValues();
		usbSendData[1] = missileCmd;
		// USBへ送信。
		if (usb_control_msg(
			udh,
			USB_ENDPOINT_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			HID_REPORT_SET,
			0,
			0,
			(char*)usbSendData,
			sizeof(usbSendData),
			USB_TIMEOUT) < 0)
		{
			// USB送信エラー。
			printf("usb write error.\n");
		} 
		usleep(10000);
		if (legInit == 1)
		{
			legInit = 0;
			// 左右脚の初期位置への移動。
			for (i = 0; i < 2; i++)
			{
				BrickPi.MotorSpeed[motor[i]] = 100;
				while (1)
				{
					BrickPiUpdateValues();
					usleep(10000);
					if (BrickPi.Sensor[sensor[i]] > 0)
					{
						BrickPi.MotorSpeed[motor[i]] = 0;
						BrickPi.Encoder[motor[i]] = 0;
						break;
					}
				}
			}
		}
		lastWalkCmd = walkCmd;
	}
	/*
	// キャプチャ解放。
	cvReleaseCapture(&capture);
	// ウィンドウ破棄。
	cvDestroyWindow("Capture");
	*/
	// USBデバイスクローズ。
	usb_close(udh);

	return 0;
}