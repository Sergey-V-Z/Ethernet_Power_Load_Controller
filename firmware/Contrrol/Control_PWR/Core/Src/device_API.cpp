/*
 * device_API.cpp
 *
 *  Created on: 3 июл. 2023 г.
 *      Author: Ierixon-HP
 */
#include "flash_spi.h"
#include "LED.h"
#include "lwip.h"
using namespace std;
#include <string>
#include "api.h"
#include <iostream>
#include <vector>
#include "hcsr04_driver.h"
#include "device_API.h"

/*variables ---------------------------------------------------------*/
extern settings_t settings;
extern I2C_HandleTypeDef hi2c1;
extern flash mem_spi;
//структуры для netcon
extern struct netif gnetif;

//Флаги для разбора сообщения
string f_cmd("C");
string f_addr("A");
string f_datd("D");
string f_datd1("N");
string delim("x");

string Сommand_execution(string in_str){
	// Парсинг
	vector<string> arr_msg;
	vector<mesage_t> arr_cmd;
	size_t prev = 0;
	size_t next;
	size_t delta = delim.length();
	bool errMSG = false;

	//разбить на сообщения
	while( ( next = in_str.find( delim, prev ) ) != string::npos ){
		arr_msg.push_back( in_str.substr( prev, (next +1)-prev ) );
		prev = next + delta;
	}
	//arr_msg.push_back( in_str.substr( prev ) );
	if(arr_msg.size() != 0){
		//занести сообщения в структуру
		int count_msg = arr_msg.size();
		for (int i = 0; i < count_msg; ++i) {
			prev = 0;
			next = 0;
			size_t posC = 0;
			size_t posA = 0;
			size_t posD = 0;
			size_t posN = 0;
			size_t posx = 0;
			mesage_t temp_msg;

			// выделение комманды
			delta = f_cmd.length();
			next = arr_msg[i].find(f_cmd);
			posC = next;
			if(next == string::npos){
				//Ошибка
				temp_msg.err = "wrong format in C flag";
				errMSG = true;
				arr_cmd.push_back(temp_msg);
				continue;

			}
			prev = next + delta;

			// выделение адреса
			delta = f_addr.length();
			next = arr_msg[i].find(f_addr, prev);
			posA = next;
			if(next == string::npos){
				//Ошибка
				temp_msg.err = "wrong format in A flag";
				errMSG = true;
				arr_cmd.push_back(temp_msg);
				continue;
			}
			prev = next + delta;

			// выделение данных
			delta = f_datd.length();
			next = arr_msg[i].find(f_datd, prev);
			posD = next;
			if(next == string::npos){
				//Ошибка
				temp_msg.err = "wrong format in D flag";
				errMSG = true;
				arr_cmd.push_back(temp_msg);
				continue;
			}
			prev = next + delta;

			// выделение данных 1
			delta = f_datd1.length();
			next = arr_msg[i].find(f_datd1, prev);
			posN = next;
			if(next == string::npos){
				//Ошибка
				temp_msg.err = "wrong format in N flag";
				errMSG = true;
				arr_cmd.push_back(temp_msg);
				continue;
			}
			prev = next + delta;

			// выделение данных
			delta = delim.length();
			next = arr_msg[i].find(delim, prev);
			posx = next;
			if(next == string::npos){
				//Ошибка
				temp_msg.err = "wrong format in x flag";
				errMSG = true;
				arr_cmd.push_back(temp_msg);
				continue;
			}

			bool isNum;
			// from flag "C" to flag "A" is a number ?
			isNum = isNumeric(arr_msg[i].substr(posC +1, (posA -1) - posC));

			// chain, check parameters
			if(isNum){
				temp_msg.cmd = (uint32_t)stoi(arr_msg[i].substr(posC +1, (posA -1) - posC)); // get number from string
				// from flag "A" to flag "D" is a number ?
				isNum = isNumeric(arr_msg[i].substr(posA +1, (posD -1) - posA));

				if(isNum){
					temp_msg.addres_var = (uint32_t)stoi(arr_msg[i].substr(posA +1, (posD -1) - posA)); // get number from string
					// from flag "D" to flag "N" is a number ?
					isNum = isNumeric(arr_msg[i].substr(posD +1, (posN -1) - posD));

					if(isNum){
						temp_msg.data_in = (uint32_t)stoi(arr_msg[i].substr(posD +1, (posN -1) - posD)); // get number from string
						// from flag "N" to flag "x" is a number ?
						isNum = isNumeric(arr_msg[i].substr(posN +1, (posx -1) - posN));

						if(isNum){
							temp_msg.data_in1 = (uint32_t)stoi(arr_msg[i].substr(posN +1, (posx -1) - posN)); // get number from string
						}
						else{
							temp_msg.err = "err after N is not number";
							errMSG = true;
						}
					}
					else{
						temp_msg.err = "err after D is not number";
						errMSG = true;
					}
				}
				else{
					temp_msg.err = "err after A is not number";
					errMSG = true;
				}
			}
			else{
				temp_msg.err = "err after C is not number";
				errMSG = true;
			}

			arr_cmd.push_back(temp_msg);
		}
	}
	else{
		mesage_t temp_msg;
		temp_msg.err = "err format message";
		arr_cmd.push_back(temp_msg);
		errMSG = true;
	}
	// Закончили парсинг

	/*-----------------------------------------------------------------------------------------------------------------------------*/
	//Выполнение комманд
	int count_cmd = arr_cmd.size();
	int ret = 0;
	if(!errMSG){
		for (int i = 0; i < count_cmd; ++i) {

			switch (arr_cmd[i].cmd) {
			case 1: // Add dev in cell
			ret = set_i2c_dev(arr_cmd[i].addres_var, arr_cmd[i].data_in, arr_cmd[i].data_in1);
			switch (ret) {
			case 1:
				arr_cmd[i].err = "out of range after D";
				arr_cmd[i].f_bool = true;
				break;
			case 2:
				arr_cmd[i].err = "out of range after N";
				arr_cmd[i].f_bool = true;
				break;
			case 3:
				arr_cmd[i].err = "out of range after A";
				arr_cmd[i].f_bool = true;
				break;
			case 4:
				arr_cmd[i].err = "err not empty cell";
				arr_cmd[i].f_bool = true;
				break;
			default:
				arr_cmd[i].err = "OK";
				break;
			}

			break;
			case 2: // Delet dev
				del_i2c_dev(arr_cmd[i].data_in1);
				arr_cmd[i].err = "OK";
				break;
			case 3: // Chanel on/off
				settings.Global_I2C[arr_cmd[i].data_in1].i2c_addr.led_Sett.On_off = arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 4: // PWM Chanel
				settings.Global_I2C[arr_cmd[i].data_in1].i2c_addr.led_Sett.PWM_out = arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 5: // PWM read
				arr_cmd[i].data_out =  settings.Global_I2C[arr_cmd[i].data_in1].i2c_addr.led_Sett.PWM;
				arr_cmd[i].need_resp = true;
				arr_cmd[i].err = "OK";
				break;
			case 6:// Curent
				arr_cmd[i].data_out =  settings.Global_I2C[arr_cmd[i].data_in1].i2c_addr.led_Sett.Current;
				arr_cmd[i].need_resp = true;
				arr_cmd[i].err = "OK";
				break;
			case 7:// is on
				arr_cmd[i].data_out =  settings.Global_I2C[arr_cmd[i].data_in1].i2c_addr.led_Sett.IsOn;
				arr_cmd[i].need_resp = true;
				arr_cmd[i].err = "OK";
				break;
			case 8: // save
				mem_spi.W25qxx_EraseSector(0);
				osDelay(5);
				mem_spi.Write(settings);
				arr_cmd[i].err = "OK";
				break;
			case 9:// load in flash
				settings.IP_end_from_settings = (uint8_t)arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 10: // Reboot
				if(arr_cmd[i].data_in){
					NVIC_SystemReset();
				}
				arr_cmd[i].err = "OK";
				break;
			case 11: // DHCP
				settings.DHCPset = (uint8_t)arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 12: // IP
				settings.saveIP.ip[arr_cmd[i].addres_var] = arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 13: // MASK
				settings.saveIP.mask[arr_cmd[i].addres_var] = arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 14: // GW
				settings.saveIP.gateway[arr_cmd[i].addres_var] = arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 15: // MAC
				settings.MAC[arr_cmd[i].addres_var] = arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 16:
				/*										Sensor2.change_settings = true; // включение режима настроек
														Sensor2.Depth = arr_cmd[i].data_in;
														Sensor2.change_settings = false; // выключение режима настроек
														arr_cmd[i].err = "OK";
				 */
				arr_cmd[i].err = "Empty cmd";
				arr_cmd[i].f_bool = true;
				break;
			default:
				arr_cmd[i].err = "Command does not exist";
				arr_cmd[i].f_bool = true;
				break;
			}
		}
	}
	/*-----------------------------------------------------------------------------------------------------------------------------*/
	//Формируем ответ
	string resp;
	if(!errMSG){
		for (int i = 0; i < count_cmd; ++i) {

			if(arr_cmd[i].f_bool == false){
				resp.append(f_cmd + to_string(arr_cmd[i].cmd));
				if(arr_cmd[i].need_resp){
					resp.append(f_datd + to_string(arr_cmd[i].data_out));
				}else{
					resp.append(" " + arr_cmd[i].err);
				}
				resp.append(delim);
			}
			else{
				resp.append(arr_cmd[i].err);
			}

		}
	}
	else{
		resp.append(arr_cmd[0].err);
	}

	return resp;
}


bool isNumeric(std::string const &str)
{
	char* p;
	strtol(str.c_str(), &p, 10);
	return *p == 0;
}
