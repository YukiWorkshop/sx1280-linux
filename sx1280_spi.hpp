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
#include <pthread.h>
#include "sx1280.hpp"

namespace YukiWorkshop::Drivers::Semtech {
/*!
 * \brief Actual implementation of a SX1280 radio
 */
	class SX1280_SPI : public SX1280 {
	public:

		SX1280_SPI(const std::string& __spidev_path, GPIO::Device& __gpio_iface,
			    uint32_t __busy_pin, uint32_t __rst_pin, uint32_t __nss_pin,
			    int32_t __dio1_pin = -1, int32_t __dio2_pin = -1, int32_t __dio3_pin = -1,
			    const RadioCallbacks_t& callbacks = {});

		virtual ~SX1280_SPI() = default;

		void SetDebug(bool __enabled) {
			RadioNss.debug = RadioReset.debug = BUSY.debug = RadioGpio.debug = __enabled;
			Debug = __enabled;
		}

		void StartIrqHandler(int __prio = 50);

		void StopIrqHandler();

		/*!
		 * \brief Soft resets the radio
		 */
		virtual void Reset();

		/*!
		 * \brief Wakes up the radio
		 */
		virtual void Wakeup();

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

		SPPI RadioSpi;
		GPIO::Device& RadioGpio;

		GPIO::LineSingle RadioNss;
		GPIO::LineSingle RadioReset;                        //!< The reset pin connected to the radio
		GPIO::LineSingle BUSY;                              //!< The pin connected to BUSY

		int32_t DIOPins[3];

		std::mutex IOLock;
		std::thread IrqThread;

		bool Debug = false;

		/*!
		 * \brief Sets the callback functions to be run on DIO1..3 interrupt
		 *
		 * \param [in]  irqHandler    A function pointer of the function to be run on every DIO interrupt
		 */
		virtual void IoIrqInit(const std::function<void()>& irqHandler);
	};
}
