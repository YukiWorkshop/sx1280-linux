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

#include "sx1280_uart.hpp"

/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */
#define WaitOnBusy( )          while(BUSY.read()){ usleep(1 * 1000); }

using namespace YukiWorkshop::Drivers::Semtech;

SX1280_UART::SX1280_UART(const std::string &__tty_path, YukiWorkshop::GPIO::Device &__gpio_iface, uint32_t __busy_pin,
			 uint32_t __rst_pin, uint32_t __ctsn_pin, int32_t __dio1_pin, int32_t __dio2_pin,
			 int32_t __dio3_pin, const RadioCallbacks_t &callbacks) :
	RadioGpio(__gpio_iface),
	DIOPins{__dio1_pin, __dio2_pin, __dio3_pin},
	BUSY(__gpio_iface.line(__busy_pin, GPIO::LineMode::Input, 0, "SX1280 BUSY")),
	RadioCtsn(__gpio_iface.line(__ctsn_pin, GPIO::LineMode::Output, 1, "SX1280 CTSN")),
	RadioReset(__gpio_iface.line(__rst_pin, GPIO::LineMode::Output, 1, "SX1280 NRESET")),
	SX1280(callbacks) {
	path_tty = __tty_path;

	fd_tty = open(__tty_path.c_str(), O_RDWR);
	assert(fd_tty > 0);

	RadioCtsn.debug = true;
	RadioReset.debug = true;
	BUSY.debug = true;
	RadioGpio.debug = true;

	RadioCtsn.write(0);
}

SX1280_UART::~SX1280_UART() {
	if (fd_tty > 0)
		close(fd_tty);
};

void SX1280_UART::UartInit() {
	std::string cmd = "stty -F " + path_tty + " raw -echo 115200 parenb -parodd";
	system(cmd.c_str());
}

void SX1280_UART::IoIrqInit(const std::function<void()>& irqHandler) {

	UartInit();

	for (auto it : DIOPins) {
		if (it != -1)
			RadioGpio.on_event(it, GPIO::LineMode::Input, GPIO::EventMode::RisingEdge,
					   [this, &irqHandler](GPIO::EventType t, uint64_t){
						   if (t == GPIO::EventType::RisingEdge)
							   irqHandler();
					   });
	}
}

void SX1280_UART::Reset() {
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

void SX1280_UART::Wakeup() {
	std::lock_guard lg(IOLock);

	//Don't wait for BUSY here

	printf("Wakeup\n");


	tty_putc(RADIO_GET_STATUS);
	tty_getc();

	// Wait for chip to be ready.
	WaitOnBusy();

	printf("Wakeup done\n");
}

void SX1280_UART::WriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	std::lock_guard lg(IOLock);

	printf("WriteCommand: 0x%02x %u\n", command, size);

	WaitOnBusy();

	tty_putc(command);

	if (size) {
		tty_putc(size);
		for (uint16_t i=0; i<size; i++) {
			tty_putc(buffer[i]);
		}
	}

	printf("WriteCommand: send done\n");


	if (command != RADIO_SET_SLEEP) {
		WaitOnBusy();
	}

	printf("WriteCommand: wait done\n");

}

void SX1280_UART::ReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	std::lock_guard lg(IOLock);

	WaitOnBusy();

	tty_putc(command);

	// Behavior on the UART is different depending of the opcode command
	if( ( command == RADIO_GET_PACKETTYPE ) ||
	    ( command == RADIO_GET_RXBUFFERSTATUS ) ||
	    ( command == RADIO_GET_RSSIINST ) ||
	    ( command == RADIO_GET_PACKETSTATUS ) ||
	    ( command == RADIO_GET_IRQSTATUS ) )
	{
		/*
		 * TODO : Check size size in UART (uint8_t in putc)
		 */
		tty_putc(size);
	}

	tty_read(buffer, size);

	WaitOnBusy();
}

void SX1280_UART::WriteRegister(uint16_t address, uint8_t *buffer, uint16_t size )
{
	std::lock_guard lg(IOLock);

	printf("WriteRegister: 0x%04x %u\n", address, size);

	WaitOnBusy();


	uint16_t addr = address;
	uint16_t i = 0;
	for( addr = address; ( addr + 255 ) < ( address + size ); )
	{
		tty_putc( RADIO_WRITE_REGISTER );
		tty_putc( ( addr & 0xFF00 ) >> 8 );
		tty_putc( addr & 0x00FF );
		tty_putc( 255 );
		for( uint16_t lastAddr = addr + 255 ; addr < lastAddr; i++, addr++ )
		{
			tty_putc( buffer[i] );
		}
	}
	tty_putc( RADIO_WRITE_REGISTER );
	tty_putc( ( addr & 0xFF00 ) >> 8 );
	tty_putc( addr & 0x00FF );
	tty_putc( address + size - addr );

	for( ; addr < ( address + size ); addr++, i++ ) {
		tty_putc( buffer[i] );
	}

	printf("WriteRegister: send done\n");


	WaitOnBusy();

	printf("WriteRegister: Wait done\n");

}

void SX1280_UART::WriteRegister(uint16_t address, uint8_t value )
{
	WriteRegister(address, &value, 1);
}

void SX1280_UART::ReadRegister(uint16_t address, uint8_t *buffer, uint16_t size) {
	std::lock_guard lg(IOLock);

	WaitOnBusy();


	uint16_t addr = address;
	uint16_t i = 0;
	for( addr = address; ( addr + 255 ) < ( address + size ); )
	{
		tty_putc( RADIO_READ_REGISTER );
		tty_putc( ( addr & 0xFF00 ) >> 8 );
		tty_putc( addr & 0x00FF );
		tty_putc( 255 );

		for( uint16_t lastAddr = addr + 255 ; addr < lastAddr; i++, addr++ )
		{
			buffer[i] = tty_getc( );
		}
	}
	tty_putc( RADIO_READ_REGISTER );
	tty_putc( ( addr & 0xFF00 ) >> 8 );
	tty_putc( addr & 0x00FF );
	tty_putc( address + size - addr );

	for( ; addr < ( address + size ); addr++, i++ )
	{
		buffer[i] = tty_getc( );
	}

	WaitOnBusy();
}

uint8_t SX1280_UART::ReadRegister(uint16_t address) {
	uint8_t data;

	ReadRegister( address, &data, 1 );
	return data;
}

void SX1280_UART::WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
	std::lock_guard lg(IOLock);

	WaitOnBusy();

	tty_putc(RADIO_WRITE_BUFFER);
	tty_putc(offset);
	tty_putc(size);


	for( uint16_t i = 0; i < size; i++ ) {
		tty_putc( buffer[i] );
	}

	WaitOnBusy();
}

void SX1280_UART::ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
	std::lock_guard lg(IOLock);

	WaitOnBusy();

	tty_putc(RADIO_READ_BUFFER);
	tty_putc(offset);
	tty_putc(size);

	for (uint16_t i = 0; i < size; i++) {
		buffer[i] = tty_getc();
	}

	WaitOnBusy();
}

uint8_t SX1280_UART::GetDioStatus() {
	return 0;
}


