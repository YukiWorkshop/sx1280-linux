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

#pragma once

#include <cassert>
#include "sx1280.hpp"

namespace YukiWorkshop::Drivers::Semtech {
/*!
 * \brief Actual implementation of a SX1280 radio
 */
	class SX1280_UART : public SX1280 {
	public:

		SX1280_UART(const std::string& __tty_path, GPIO::Device& __gpio_iface,
			    uint32_t __busy_pin, uint32_t __rst_pin, uint32_t __ctsn_pin,
			    int32_t __dio1_pin = -1, int32_t __dio2_pin = -1, int32_t __dio3_pin = -1,
			    const RadioCallbacks_t& callbacks = {});

		virtual ~SX1280_UART(void);

		void RunIrqHandler() {
			RadioGpio.run_eventlistener();
		}

		void StopIrqHandler() {
			RadioGpio.stop_eventlistener();
		}

		/*!
		 * \brief Soft resets the radio
		 */
		virtual void Reset(void);

		/*!
		 * \brief Wakes up the radio
		 */
		virtual void Wakeup(void);

		/*!
		 * \brief Set the SPI Speed
		 *
		 * \param [in]  spiSpeed      Speed of the SPI in Hz
		 */
		void SetSpiSpeed(uint32_t spiSpeed);

		/*!
		 * \brief Send a command that write data to the radio
		 *
		 * \param [in]  opcode        Opcode of the command
		 * \param [in]  buffer        Buffer to be send to the radio
		 * \param [in]  size          Size of the buffer to send
		 */
		virtual void WriteCommand(RadioCommands_t opcode, uint8_t *buffer, uint16_t size);

		/*!
		 * \brief Send a command that read data from the radio
		 *
		 * \param [in]  opcode        Opcode of the command
		 * \param [out] buffer        Buffer holding data from the radio
		 * \param [in]  size          Size of the buffer
		 */
		virtual void ReadCommand(RadioCommands_t opcode, uint8_t *buffer, uint16_t size);

		/*!
		 * \brief Write data to the radio memory
		 *
		 * \param [in]  address       The address of the first byte to write in the radio
		 * \param [in]  buffer        The data to be written in radio's memory
		 * \param [in]  size          The number of bytes to write in radio's memory
		 */
		virtual void WriteRegister(uint16_t address, uint8_t *buffer, uint16_t size);

		/*!
		 * \brief Write a single byte of data to the radio memory
		 *
		 * \param [in]  address       The address of the first byte to write in the radio
		 * \param [in]  value         The data to be written in radio's memory
		 */
		virtual void WriteRegister(uint16_t address, uint8_t value);

		/*!
		 * \brief Read data from the radio memory
		 *
		 * \param [in]  address       The address of the first byte to read from the radio
		 * \param [out] buffer        The buffer that holds data read from radio
		 * \param [in]  size          The number of bytes to read from radio's memory
		 */
		virtual void ReadRegister(uint16_t address, uint8_t *buffer, uint16_t size);

		/*!
		 * \brief Read a single byte of data from the radio memory
		 *
		 * \param [in]  address       The address of the first byte to write in the
		 *                            radio
		 *
		 * \retval      value         The value of the byte at the given address in
		 *                            radio's memory
		 */
		virtual uint8_t ReadRegister(uint16_t address);

		/*!
		 * \brief Write data to the buffer holding the payload in the radio
		 *
		 * \param [in]  offset        The offset to start writing the payload
		 * \param [in]  buffer        The data to be written (the payload)
		 * \param [in]  size          The number of byte to be written
		 */
		virtual void WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size);

		/*!
		 * \brief Read data from the buffer holding the payload in the radio
		 *
		 * \param [in]  offset        The offset to start reading the payload
		 * \param [out] buffer        A pointer to a buffer holding the data from the radio
		 * \param [in]  size          The number of byte to be read
		 */
		virtual void ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size);

		/*!
		 * \brief Returns the status of DIOs pins
		 *
		 * \retval      dioStatus     A byte where each bit represents a DIO state:
		 *                            [ DIO3 | DIO2 | DIO1 | BUSY ]
		 */
		virtual uint8_t GetDioStatus(void);

	protected:
		int fd_tty = -1;
		std::string path_tty;

		GPIO::Device& RadioGpio;

		GPIO::LineSingle RadioReset;                        //!< The reset pin connected to the radio
		GPIO::LineSingle BUSY;                              //!< The pin connected to BUSY
		GPIO::LineSingle RadioCtsn;

		int32_t DIOPins[3];

		std::mutex IOLock;
		std::thread IrqThread;

		uint8_t tty_getc() {
			uint8_t c;
			read(fd_tty, &c, 1);
			return c;
		}

		void tty_putc(uint8_t c) {
			write(fd_tty, &c, 1);
		}

		ssize_t tty_write(const void *__buf, size_t __n) {
			size_t written = 0;

			while (written < __n) {
				ssize_t rc = write(fd_tty, (uint8_t *) __buf + written, __n - written);
				if (rc > 0) {
					written += rc;
				} else if (rc == 0) {
					return written;
				} else {
					return -1;
				}
			}

			return written;
		}

		ssize_t tty_read(const void *__buf, size_t __n) {
			size_t readed = 0;

			while (readed < __n) {
				ssize_t rc = read(fd_tty, (uint8_t *) __buf + readed, __n - readed);
				if (rc > 0) {
					readed += rc;
				} else if (rc == 0) {
					return readed;
				} else {
					return -1;
				}
			}

			return readed;
		}

		/*!
		 * \brief Initializes UART object used to communicate with the radio
		 */
		virtual void UartInit();

		/*!
		 * \brief Sets the callback functions to be run on DIO1..3 interrupt
		 *
		 * \param [in]  irqHandler    A function pointer of the function to be run on every DIO interrupt
		 */
		virtual void IoIrqInit(const std::function<void()>& irqHandler);
	};
}