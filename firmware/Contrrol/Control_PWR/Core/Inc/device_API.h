/*
 * device_API.h
 *
 *  Created on: 3 июл. 2023 г.
 *      Author: Ierixon-HP
 */

#ifndef INC_DEVICE_API_H_
#define INC_DEVICE_API_H_

/* Typedef -----------------------------------------------------------*/
struct mesage_t{
	uint32_t cmd;
	uint32_t addres_var;
	uint32_t data_in;
	uint32_t data_in1;
	bool need_resp = false;
	bool data_in_is;
	uint32_t data_out;
	string err; // сообщение клиенту об ошибке в сообщении
	bool f_bool = false; // наличие ошибки в сообшении
};



string Сommand_execution(string in_str);
bool isNumeric(std::string const &str);

#endif /* INC_DEVICE_API_H_ */
