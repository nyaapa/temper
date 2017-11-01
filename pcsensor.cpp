
/*
 * pcsensor.cpp by nyaapa@ 
 * pcsensor.c by Philipp Adelt (c) 2012 (info@philipp.adelt.net)
 * based on Juan Carlos Perez (c) 2011 (cray@isp-sl.com)
 * based on Temper.c by Robert Kavaler (c) 2009 (relavak.com)
 * All rights reserved.
 *
 * Temper driver for linux. This program can be compiled either as a library
 * or as a standalone program (-DUNIT_TEST). The driver will work with some
 * TEMPer usb devices from RDing (www.PCsensor.com).
 *
 * This driver works with USB devices presenting ID 0c45:7401.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 * 
 * THIS SOFTWARE IS PROVIDED BY Philipp Adelt (and other contributors) ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Philipp Adelt (or other contributors) BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <usb.h>
#include <errno.h>
#include <cstring>
#include <cstddef>
#include <iostream>
#include <exception>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <iterator>
#include <numeric>
#include <array>

constexpr uint VENDOR_ID = 0x0c45;
constexpr uint PRODUCT_ID = 0x7401;

constexpr uint INTERFACE1 = 0x00;
constexpr uint INTERFACE2 = 0x01;

constexpr int request_lenght = 8;
constexpr int endpoint_in = 0x82; /* endpoint 0x81 address for IN */
constexpr int endpoint_out = 0x00; /* endpoint 1 address for OUT */
constexpr int timeout = 5000; /* timeout in ms */

constexpr unsigned char ini_control[] = { 0x01, 0x01 };
constexpr unsigned char temperature[] = { 0x01, 0x80, 0x33, 0x01, 0x00, 0x00, 0x00, 0x00 };
constexpr unsigned char ini_1[] = { 0x01, 0x82, 0x77, 0x01, 0x00, 0x00, 0x00, 0x00 };
constexpr unsigned char ini_2[] = { 0x01, 0x86, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00 };

/* Even within the same VENDOR_ID / PRODUCT_ID, there are hardware variations
 * which we can detect by reading the USB product ID string. This determines
 * where the temperature offset is stored in the USB read buffer. */
constexpr size_t default_temp_offset = 2;
const static std::unordered_map<std::string, size_t> product_temp_offset{
	{ "TEMPer1F_V1.3", 4 },
};

size_t get_temp_offset(usb_dev_handle *handle) {
	char prod_id[256];

	memset(prod_id, 0, sizeof(prod_id));
	auto len = usb_get_string_simple(handle, usb_device(handle)->descriptor.iProduct, prod_id, sizeof(prod_id)-1);
	if (len < 0)
		std::cerr << "Couldn't read iProduct string\n";
	else
		std::cerr << "iProduct: " << prod_id << "\n";

	if (auto it = product_temp_offset.find(prod_id); it != product_temp_offset.end())
		return it->second;

	return default_temp_offset;
}

usb_dev_handle *find_lvr_winusb() {
	for (auto bus = usb_busses; bus; bus = bus->next) {
		for (auto dev = bus->devices; dev; dev = dev->next) {
			if (dev->descriptor.idVendor == VENDOR_ID && dev->descriptor.idProduct == PRODUCT_ID) {
				std::cerr << "lvr_winusb with Vendor Id: " << VENDOR_ID << " and Product Id: " << PRODUCT_ID << " found.\n";
				auto handle = usb_open(dev);

				if (!handle)
					throw std::runtime_error("Could not open USB device");
				return handle;
			}
		}
	}

	throw std::runtime_error("Couldn't find the USB device");
}

void usb_detach(usb_dev_handle *lvr_winusb, int interface) {
	if(usb_detach_kernel_driver_np(lvr_winusb, interface)) {
		if(errno == ENODATA) {
			std::cerr << "Device already detached\n";
		} else {
			std::cerr << "Detach failed: " << strerror(errno) << "[" << errno << "]\n";
			std::cerr << "Continuing anyway\n";
		}
	} else {
		std::cerr << "Detach successful\n";
	}
}

usb_dev_handle* setup_libusb_access() {
#ifndef NDEBUG
	usb_set_debug(255);
#else
	usb_set_debug(0);
#endif
	usb_init();
	usb_find_busses();
	usb_find_devices();

	auto lvr_winusb = find_lvr_winusb();

	usb_detach(lvr_winusb, INTERFACE1);
	usb_detach(lvr_winusb, INTERFACE2);

	if (usb_set_configuration(lvr_winusb, 0x01) < 0)
		throw std::runtime_error("Could not set configuration 1");

	// Microdia tiene 2 interfaces
	if (usb_claim_interface(lvr_winusb, INTERFACE1) < 0)
		throw std::runtime_error("Could not claim interface");

	if (usb_claim_interface(lvr_winusb, INTERFACE2) < 0)
		throw std::runtime_error("Could not claim interface");

	return lvr_winusb;
}

template<typename T>
void control_transfer(usb_dev_handle *dev, const T& pquestion, int msg_value, int msg_index) {
	char question[sizeof(pquestion)];

	memcpy(question, pquestion, sizeof(pquestion));
	
	auto response = usb_control_msg(dev, 0x21, 0x09, msg_value, msg_index, (char *) question, sizeof(pquestion), timeout);
	if (response < 0)
		throw std::runtime_error(std::string("USB control write failed: ") + strerror(errno));

	#ifndef NDEBUG
		for (auto el : question)
			fprintf(stderr, "%02x ", el & 0xFF);
		fprintf(stderr, "\n");
	#endif
}

void interrupt_transfer(usb_dev_handle *dev) {
	char answer[request_lenght];
	char question[request_lenght];
	std::iota(std::begin(question), std::end(question), 0);

	auto response = usb_interrupt_write(dev, endpoint_out, question, request_lenght, timeout);
	if (response < 0)
		throw std::runtime_error(std::string("USB interrupt write failed: ") + strerror(errno));

	response = usb_interrupt_read(dev, endpoint_in, answer, request_lenght, timeout);
	if (response != request_lenght)
		throw std::runtime_error(std::string("USB interrupt write failed: ") + strerror(errno));

#ifndef NDEBUG
	for (int i = 0; i < request_lenght; i++)
		std::cerr << question[i] << " => " << answer[i] << "\n";
#endif

	usb_release_interface(dev, 0);
}

std::array<unsigned char, request_lenght> interrupt_read(usb_dev_handle *dev) {
	std::array<unsigned char, request_lenght> answer;
	answer.fill(0);

	auto response = usb_interrupt_read(dev, 0x82, reinterpret_cast<char*>(answer.begin()), request_lenght, timeout);
	if (response != request_lenght)
		throw std::runtime_error(std::string("USB interrupt read failed: ") + strerror(errno));

#ifndef NDEBUG
	for (auto el : answer)
		fprintf(stderr, "%02x ", el & 0xFF);
	fprintf(stderr, "\n");
#endif

	return answer;
}

uint interrupt_read_mega_kelvin(usb_dev_handle *dev, size_t temp_offset) {
	auto answer = interrupt_read(dev);

	/* Temperature in C is a 16-bit signed fixed-point number, big-endian */
	return (answer[temp_offset] * 256 + answer[temp_offset + 1]) * 1e6 / 256 + 273150000u;
}

int main(int, char**) {
	auto lvr_winusb = setup_libusb_access();
	auto temp_offset = get_temp_offset(lvr_winusb);

	control_transfer(lvr_winusb, ini_control, 0x0201, 0x00);

	control_transfer(lvr_winusb, temperature, 0x0200, 0x01);
	interrupt_read(lvr_winusb);

	control_transfer(lvr_winusb, ini_1, 0x0200, 0x01);
	interrupt_read(lvr_winusb);

	control_transfer(lvr_winusb, ini_2, 0x0200, 0x01);
	interrupt_read(lvr_winusb);
	interrupt_read(lvr_winusb);

	control_transfer(lvr_winusb, temperature, 0x0200, 0x01);

	std::cout << interrupt_read_mega_kelvin(lvr_winusb, temp_offset) << "MK\n";

	usb_release_interface(lvr_winusb, INTERFACE1);
	usb_release_interface(lvr_winusb, INTERFACE2);

	usb_reset(lvr_winusb);

	return 0;
}