/*
    This file is part of SX1280 Linux driver.
    Copyright (C) 2020 ReimuNotMoe

    This program is based on sx1280-driver from Semtech S.A.,
    see LICENSE-SEMTECH.txt for details.

    Original maintainer of sx1280-driver: Miguel Luis, Gregory Cristian
    and Matthieu Verdy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "sx1280_spi.hpp"

/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */
#define WaitOnBusy( )          while(BUSY.read()){ usleep(10); }

using namespace YukiWorkshop::Drivers::Semtech;

SX1280_SPI::SX1280_SPI(const std::string &__spidev_path, YukiWorkshop::GPIO::Device &__gpio_iface, uint32_t __busy_pin,
		       uint32_t __rst_pin, uint32_t __nss_pin, int32_t __dio1_pin, int32_t __dio2_pin,
		       int32_t __dio3_pin, const RadioCallbacks_t &callbacks) :
	RadioSpi(__spidev_path, SPI_MODE_0|SPI_NO_CS, 8, 8000000),
	RadioGpio(__gpio_iface),
	DIOPins{__dio1_pin, __dio2_pin, __dio3_pin},
	BUSY(__gpio_iface.line(__busy_pin, GPIO::LineMode::Input, 0, "SX1280 BUSY")),
	RadioNss(__gpio_iface.line(__nss_pin, GPIO::LineMode::Output, 1, "SX1280 NSS")),
	RadioReset(__gpio_iface.line(__rst_pin, GPIO::LineMode::Output, 1, "SX1280 NRESET")),
	SX1280(callbacks) {

}

void SX1280_SPI::SetSpiSpeed(uint32_t spiSpeed) {
	RadioSpi.set_max_speed_hz(spiSpeed);
}

void SX1280_SPI::IoIrqInit(const std::function<void()>& irqHandler) {
	for (auto it : DIOPins) {
		if (it != -1)
			RadioGpio.on_event(it, GPIO::LineMode::Input, GPIO::EventMode::RisingEdge,
					   [this, &irqHandler](GPIO::EventType t, uint64_t){
						   if (t == GPIO::EventType::RisingEdge)
							   irqHandler();
					   });
	}
}

void SX1280_SPI::Reset() {
	std::lock_guard lg(IOLock);


	RadioReset.set_mode(GPIO::LineMode::Output, 1, "SX1280 NRESET (Out)");
	usleep(50 * 1000);

	RadioReset.write(0);
	usleep(50 * 1000);
	RadioReset.write(1);
	usleep(50 * 1000);

	RadioReset.set_mode(GPIO::LineMode::Input, 0, "SX1280 NRESET (In)");
	usleep(50 * 1000);

	BUSY.read();
}

void SX1280_SPI::Wakeup() {
	std::lock_guard lg(IOLock);

	// Don't wait for BUSY here

	if (Debug)
		printf("SX1280: Wakeup\n");

	uint8_t buf[2] = {RADIO_GET_STATUS, 0};

	RadioNss.write(0);

	RadioSpi.write(buf, 2);

	RadioNss.write(1);

	// Wait for chip to be ready.
	WaitOnBusy();

	if (Debug)
		printf("SX1280: Wakeup done\n");
}

void SX1280_SPI::WriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	std::lock_guard lg(IOLock);

	if (Debug)
		printf("SX1280: WriteCommand: 0x%02x %u\n", command, size);

	WaitOnBusy();

	RadioNss.write(0);

	RadioSpi.transfer(command, false);
	RadioSpi.write(buffer, size);

	RadioNss.write(1);

	if (Debug)
		printf("SX1280: WriteCommand: send done\n");


	if (command != RADIO_SET_SLEEP) {
		WaitOnBusy();
	}

	if (Debug)
		printf("SX1280: WriteCommand: wait done\n");

}

void SX1280_SPI::ReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	std::lock_guard lg(IOLock);

	WaitOnBusy();

	RadioNss.write(0);

	if( command == RADIO_GET_STATUS ) {
		buffer[0] = RadioSpi.transfer(command, false);
		RadioSpi.transfer(0, false);
		RadioSpi.transfer(0);
	} else {
		RadioSpi.transfer(command, false);
		RadioSpi.transfer(0, false);

		RadioSpi.read(buffer, size);
	}

	RadioNss.write(1);

	WaitOnBusy();
}

void SX1280_SPI::WriteRegister(uint16_t address, uint8_t *buffer, uint16_t size ) {
	std::lock_guard lg(IOLock);

	if (Debug)
		printf("SX1280: WriteRegister: 0x%04x %u\n", address, size);

	WaitOnBusy();

	RadioNss.write(0);

	RadioSpi.transfer(RADIO_WRITE_REGISTER, false);
	RadioSpi.transfer((address & 0xFF00) >> 8, false);
	RadioSpi.transfer(address & 0x00FF, false);

	RadioSpi.write(buffer, size);

	RadioNss.write(1);

	if (Debug)
		printf("SX1280: WriteRegister: send done\n");

	WaitOnBusy();

	if (Debug)
		printf("SX1280: WriteRegister: Wait done\n");
}

void SX1280_SPI::WriteRegister(uint16_t address, uint8_t value ) {
	WriteRegister(address, &value, 1);
}

void SX1280_SPI::ReadRegister(uint16_t address, uint8_t *buffer, uint16_t size) {
	std::lock_guard lg(IOLock);

	WaitOnBusy();

	RadioNss.write(0);

	uint8_t buf[4] = {RADIO_READ_REGISTER, static_cast<uint8_t>((address & 0xFF00) >> 8),
			  static_cast<uint8_t>(address & 0x00FF), 0};

	RadioSpi.write(buf, 4);

//	RadioSpi.transfer(RADIO_READ_REGISTER, false);
//	RadioSpi.transfer( (address & 0xFF00) >> 8 , false);
//	RadioSpi.transfer( address & 0x00FF , false);
//	RadioSpi.transfer( 0 , false);

	RadioSpi.read(buffer, size);

	RadioNss.write(1);

	WaitOnBusy();
}

uint8_t SX1280_SPI::ReadRegister(uint16_t address) {
	uint8_t data;

	ReadRegister( address, &data, 1 );
	return data;
}

void SX1280_SPI::WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
	std::lock_guard lg(IOLock);

	WaitOnBusy();

	RadioNss.write(0);

	RadioSpi.transfer( RADIO_WRITE_BUFFER, false);
	RadioSpi.transfer( offset, false);

	RadioSpi.write(buffer, size);

	RadioNss.write(1);

	WaitOnBusy();
}

void SX1280_SPI::ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
	std::lock_guard lg(IOLock);

	WaitOnBusy();

	RadioNss.write(0);

	RadioSpi.transfer( RADIO_READ_BUFFER, false);
	RadioSpi.transfer( offset, false);
	RadioSpi.transfer( 0, false);

	RadioSpi.read(buffer, size);

	RadioNss.write(1);

	WaitOnBusy();
}

uint8_t SX1280_SPI::GetDioStatus() {
	return 0;
}

void SX1280_SPI::StartIrqHandler(int __prio) {
	IrqThread = std::thread([this, __prio](){
		sched_param param;
		param.sched_priority = __prio;
		pthread_setschedparam(pthread_self(), SCHED_RR, &param);
		RadioGpio.run_eventlistener();
	});
}

void SX1280_SPI::StopIrqHandler() {
	RadioGpio.stop_eventlistener();
	IrqThread.join();
}


