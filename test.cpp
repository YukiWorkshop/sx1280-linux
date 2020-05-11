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
#include "sx1280_spi.hpp"

using namespace YukiWorkshop;
using namespace YukiWorkshop::Drivers::Semtech;

int main() {

	GPIO::Device g(0);

	RadioCallbacks_t cbs;
	cbs.txDone = [&]() {
		std::cout << "wow tx done!\n";
	};

	cbs.txTimeout = [&]() {
		std::cout << "wow tx timeout!\n";
	};

//	Drivers::Semtech::SX1280_UART Radio("/dev/ttyUSB0", g, 19, 6, 13, 26, -1, -1, cbs);

	Drivers::Semtech::SX1280_SPI Radio("/dev/spidev0.0", g, 19, 6, 13, 26, -1, -1, cbs);

	Radio.Reset();

	Radio.Init();
	puts("Init done");
	Radio.Wakeup();
	Radio.Wakeup();
	Radio.Wakeup();

	Radio.SetStandby( STDBY_XOSC );
	puts("SetStandby done");
	Radio.SetRegulatorMode( (RadioRegulatorModes_t)0 );
	puts("SetRegulatorMode done");
	Radio.SetLNAGainSetting(LNA_HIGH_SENSITIVITY_MODE);
	puts("SetLNAGainSetting done");
	Radio.SetRfFrequency( 2488 * 1000000UL );
	puts("SetRfFrequency done");
	Radio.SetTxParams(12, RADIO_RAMP_20_US );
	puts("SetTxParams done");

	Radio.SetBufferBaseAddresses( 0x00, 0x00 );
	PacketParams_t PacketParams;
	PacketStatus_t PacketStatus;

	ModulationParams_t ModulationParams;
	ModulationParams.PacketType = PACKET_TYPE_FLRC;
	ModulationParams.Params.Flrc.CodingRate = FLRC_CR_3_4;
	ModulationParams.Params.Flrc.BitrateBandwidth = FLRC_BR_0_325_BW_0_3;
	ModulationParams.Params.Flrc.ModulationShaping = RADIO_MOD_SHAPING_BT_OFF;

	PacketParams.PacketType = PACKET_TYPE_FLRC;
	auto &f = PacketParams.Params.Flrc;
	f.PreambleLength = PREAMBLE_LENGTH_32_BITS;
	f.SyncWordLength = FLRC_SYNCWORD_LENGTH_4_BYTE;
	f.SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_1;
	f.HeaderType = RADIO_PACKET_FIXED_LENGTH;
	f.PayloadLength = 127;
	f.CrcLength = RADIO_CRC_2_BYTES;
	f.Whitening = RADIO_WHITENING_OFF;

	ModulationParams_t ModulationParams2;
	ModulationParams2.PacketType = PACKET_TYPE_LORA;
	ModulationParams2.Params.LoRa.CodingRate = LORA_CR_4_5;
	ModulationParams2.Params.LoRa.Bandwidth = LORA_BW_1600;
	ModulationParams2.Params.LoRa.SpreadingFactor = LORA_SF5;

	PacketParams_t PacketParams2;
	PacketParams2.PacketType = PACKET_TYPE_LORA;
	auto &l = PacketParams2.Params.LoRa;
	l.PayloadLength = 253;
	l.HeaderType = LORA_PACKET_FIXED_LENGTH;
	l.PreambleLength = 12;
	l.Crc = LORA_CRC_OFF;
	l.InvertIQ = LORA_IQ_NORMAL;

	ModulationParams_t ModulationParams3;
	ModulationParams3.PacketType = PACKET_TYPE_GFSK;
	ModulationParams3.Params.Gfsk.BitrateBandwidth = GFSK_BLE_BR_2_000_BW_2_4;
	ModulationParams3.Params.Gfsk.ModulationShaping = RADIO_MOD_SHAPING_BT_OFF;
	ModulationParams3.Params.Gfsk.ModulationIndex = GFSK_BLE_MOD_IND_4_00;


	PacketParams_t PacketParams3;
	PacketParams2.PacketType = PACKET_TYPE_GFSK;
	auto &gg = PacketParams3.Params.Gfsk;
	gg.PreambleLength = PREAMBLE_LENGTH_32_BITS;
	gg.SyncWordLength = GFSK_SYNCWORD_LENGTH_4_BYTE;
	gg.SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_1;
	gg.HeaderType = RADIO_PACKET_VARIABLE_LENGTH;
	gg.PayloadLength = 127;
	gg.CrcLength = RADIO_CRC_2_BYTES;
	gg.Whitening = RADIO_WHITENING_OFF;

	Radio.SetPacketType(PACKET_TYPE_LORA);

	Radio.SetModulationParams( &ModulationParams2 );
	Radio.SetPacketParams( &PacketParams2 );

	// only used in GFSK, FLRC (4 bytes max) and BLE mode
	uint8_t sw[] = { 0xDD, 0xA0, 0x96, 0x69, 0xDD };
	Radio.SetSyncWord( 1, sw);
	// only used in GFSK, FLRC
	uint8_t crcSeedLocal[2] = {0x45, 0x67};
	Radio.SetCrcSeed( crcSeedLocal );
	Radio.SetCrcPolynomial( 0x0123 );

	std::cout << Radio.GetFirmwareVersion() << "\n";

	Radio.SetTxContinuousWave( );
	puts("SetTxContinuousWave done");
//	sleep(100);

	while (1) {
		for (size_t i = 2470; i < 2501; i++) {
			Radio.SetRfFrequency(i * 1000000UL);
//			usleep(1000 * 50);
		}
	}

	auto IrqMask = IRQ_RX_DONE | IRQ_TX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
	Radio.SetDioIrqParams(IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);

	std::thread irqthread([&](){Radio.RunIrqHandler();});
//	Radio.SetRfFrequency( 2470000000UL );
//	sleep(2);
	while (1) {
		char buf[253];
		Radio.SendPayload((uint8_t *)buf, 253, ( TickTime_t ){ RADIO_TICK_SIZE_1000_US, 10000 } );
		puts("SendPayload done");

//		usleep(3 * 1000);
//		sleep(1);
	}

	puts("setup done");
	sleep(100);

}