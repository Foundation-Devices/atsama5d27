use {
    crate::uart::{Uart, Uart1},
    core::fmt::Write,
    usb_device::bus::UsbBus,
    utralib::CSR,
};

type Console = Uart<Uart1>;

#[derive(Debug)]
pub struct Bus;

// TODO I might actually end up needing to support all 16 endpoints
// TODO For better performance, consider setting BURST_LCK to 1

impl UsbBus for Bus {
    fn alloc_ep(
        &mut self,
        ep_dir: usb_device::UsbDirection,
        ep_addr: Option<usb_device::endpoint::EndpointAddress>,
        ep_type: usb_device::endpoint::EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> usb_device::Result<usb_device::endpoint::EndpointAddress> {
        // TODO Endpoints must be configured in order (from 0 to 3) - this is why MAPD is not
        // being set
        // TODO Idea: store them here and then apply them all in enable()

        writeln!(
            Console::new(),
            "alloc_ep {:?} {:?} {:?} {:?}",
            ep_dir,
            ep_addr,
            ep_type,
            max_packet_size
        )
        .unwrap();

        if matches!(
            ep_type,
            usb_device::endpoint::EndpointType::Isochronous { .. }
        ) {
            panic!("Isochronous endpoints not supported");
        }

        let ept_size = match max_packet_size {
            1..=8 => 0,
            9..=16 => 1,
            17..=32 => 2,
            33..=64 => 3,
            65..=128 => 4,
            129..=256 => 5,
            257..=512 => 6,
            _ => panic!("Invalid max_packet_size {max_packet_size} for {ep_type:?}"),
        };

        let mut udphs = CSR::new(utralib::HW_UDPHS_BASE as *mut u32);

        let ep_index = ep_addr.map(|addr| addr.index()).unwrap_or_else(|| {
            if udphs.rf(utralib::utra::udphs::EPTCTL1_EPT_ENABL) == 0 {
                1
            } else if udphs.rf(utralib::utra::udphs::EPTCTL2_EPT_ENABL) == 0 {
                2
            } else if udphs.rf(utralib::utra::udphs::EPTCTL3_EPT_ENABL) == 0 {
                3
            } else {
                panic!("No free endpoint available for {ep_type:?} {ep_dir:?} {max_packet_size}")
            }
        });

        writeln!(Console::new(), "ep_index {}", ep_index).ok();

        let (
            f_ept_size,
            f_ept_dir,
            f_ept_type,
            f_ept_bk_number,
            f_ept_mapd,
            f_ien,
            f_en_ept_enabl,
            f_en_tx_complt,
            f_en_rxrdy_txkl,
            f_en_rx_setup,
            f_auto_valid,
        ) = match ep_index {
            0 => (
                utralib::utra::udphs::EPTCFG0_EPT_SIZE,
                utralib::utra::udphs::EPTCFG0_EPT_DIR,
                utralib::utra::udphs::EPTCFG0_EPT_TYPE,
                utralib::utra::udphs::EPTCFG0_BK_NUMBER,
                utralib::utra::udphs::EPTCFG0_EPT_MAPD,
                utralib::utra::udphs::IEN_EPT_0,
                utralib::utra::udphs::EPTCTLEN0_EPT_ENABL,
                utralib::utra::udphs::EPTCTLEN0_TX_COMPLT,
                utralib::utra::udphs::EPTCTLEN0_RXRDY_TXKL,
                utralib::utra::udphs::EPTCTLEN0_RX_SETUP,
                utralib::utra::udphs::EPTCTL0_AUTO_VALID,
            ),
            1 => (
                utralib::utra::udphs::EPTCFG1_EPT_SIZE,
                utralib::utra::udphs::EPTCFG1_EPT_DIR,
                utralib::utra::udphs::EPTCFG1_EPT_TYPE,
                utralib::utra::udphs::EPTCFG1_BK_NUMBER,
                utralib::utra::udphs::EPTCFG1_EPT_MAPD,
                utralib::utra::udphs::IEN_EPT_1,
                utralib::utra::udphs::EPTCTLEN1_EPT_ENABL,
                utralib::utra::udphs::EPTCTLEN1_TX_COMPLT,
                utralib::utra::udphs::EPTCTLEN1_RXRDY_TXKL,
                utralib::utra::udphs::EPTCTLEN1_RX_SETUP,
                utralib::utra::udphs::EPTCTL1_AUTO_VALID,
            ),
            2 => (
                utralib::utra::udphs::EPTCFG2_EPT_SIZE,
                utralib::utra::udphs::EPTCFG2_EPT_DIR,
                utralib::utra::udphs::EPTCFG2_EPT_TYPE,
                utralib::utra::udphs::EPTCFG2_BK_NUMBER,
                utralib::utra::udphs::EPTCFG2_EPT_MAPD,
                utralib::utra::udphs::IEN_EPT_2,
                utralib::utra::udphs::EPTCTLEN2_EPT_ENABL,
                utralib::utra::udphs::EPTCTLEN2_TX_COMPLT,
                utralib::utra::udphs::EPTCTLEN2_RXRDY_TXKL,
                utralib::utra::udphs::EPTCTLEN2_RX_SETUP,
                utralib::utra::udphs::EPTCTL2_AUTO_VALID,
            ),
            3 => (
                utralib::utra::udphs::EPTCFG3_EPT_SIZE,
                utralib::utra::udphs::EPTCFG3_EPT_DIR,
                utralib::utra::udphs::EPTCFG3_EPT_TYPE,
                utralib::utra::udphs::EPTCFG3_BK_NUMBER,
                utralib::utra::udphs::EPTCFG3_EPT_MAPD,
                utralib::utra::udphs::IEN_EPT_3,
                utralib::utra::udphs::EPTCTLEN3_EPT_ENABL,
                utralib::utra::udphs::EPTCTLEN3_TX_COMPLT,
                utralib::utra::udphs::EPTCTLEN3_RXRDY_TXKL,
                utralib::utra::udphs::EPTCTLEN3_RX_SETUP,
                utralib::utra::udphs::EPTCTL3_AUTO_VALID,
            ),
            _ => panic!("invalid endpoint index {ep_index}"),
        };

        writeln!(Console::new(), "ept_size {}", ept_size).ok();

        udphs.wfo(f_ept_size, ept_size);
        udphs.wfo(
            f_ept_dir,
            match ep_dir {
                usb_device::UsbDirection::Out => 0,
                usb_device::UsbDirection::In => 1,
            },
        );
        udphs.wfo(
            f_ept_type,
            match ep_type {
                usb_device::endpoint::EndpointType::Control => 0,
                usb_device::endpoint::EndpointType::Isochronous { .. } => 1,
                usb_device::endpoint::EndpointType::Bulk => 2,
                usb_device::endpoint::EndpointType::Interrupt => 3,
            },
        );
        udphs.wfo(f_ept_bk_number, 2);

        udphs.wfo(f_ien, 1);
        udphs.wfo(f_en_rx_setup, 1);
        match ep_dir {
            usb_device::UsbDirection::Out => {
                udphs.wfo(f_en_rxrdy_txkl, 1);
                udphs.wfo(f_en_tx_complt, 0);
                udphs.wfo(f_auto_valid, 1);
            }
            usb_device::UsbDirection::In => {
                udphs.wfo(f_en_rxrdy_txkl, 0);
                udphs.wfo(f_en_tx_complt, 1);
                udphs.wfo(f_auto_valid, 0);
            }
        }
        udphs.wfo(f_en_ept_enabl, 1);

        writeln!(Console::new(), "ept_mapd {}", udphs.rf(f_ept_mapd)).ok();
        if udphs.rf(f_ept_mapd) != 1 {
            panic!("mapd not set, configuration uses too much memory");
        }

        Ok(usb_device::endpoint::EndpointAddress::from_parts(
            ep_index, ep_dir,
        ))
    }

    fn enable(&mut self) {
        writeln!(Console::new(), "enable").ok();
        let mut udphs = CSR::new(utralib::HW_UDPHS_BASE as *mut u32);
        udphs.wfo(utralib::utra::udphs::CTRL_EN_UDPHS, 1);
    }

    fn reset(&self) {
        writeln!(Console::new(), "reset").ok();
        // TODO Problem: I never disable endpoints
        // Should I do that here?
        // Not sure
        // Check if the other code does it anywhere
    }

    fn set_device_address(&self, addr: u8) {
        let mut udphs = CSR::new(utralib::HW_UDPHS_BASE as *mut u32);
        udphs.wfo(utralib::utra::udphs::CTRL_DEV_ADDR, addr.into());
    }

    fn write(
        &self,
        ep_addr: usb_device::endpoint::EndpointAddress,
        buf: &[u8],
    ) -> usb_device::Result<usize> {
        let mut udphs = CSR::new(utralib::HW_UDPHS_BASE as *mut u32);
        let (
            f_dmaaddress,
            f_buff_length,
            f_end_buffit,
            f_chann_enb_command,
            f_chann_enb_status,
            f_chann_act_status,
        ) = match ep_addr.index() {
            0 => (
                utralib::utra::udphs::DMAADDRESS0_BUFF_ADD,
                utralib::utra::udphs::DMACONTROL0_BUFF_LENGTH,
                utralib::utra::udphs::DMACONTROL0_END_BUFFIT,
                utralib::utra::udphs::DMACONTROL0_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS0_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS0_CHANN_ACT,
            ),
            1 => (
                utralib::utra::udphs::DMAADDRESS1_BUFF_ADD,
                utralib::utra::udphs::DMACONTROL1_BUFF_LENGTH,
                utralib::utra::udphs::DMACONTROL1_END_BUFFIT,
                utralib::utra::udphs::DMACONTROL1_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS1_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS1_CHANN_ACT,
            ),
            2 => (
                utralib::utra::udphs::DMAADDRESS2_BUFF_ADD,
                utralib::utra::udphs::DMACONTROL2_BUFF_LENGTH,
                utralib::utra::udphs::DMACONTROL2_END_BUFFIT,
                utralib::utra::udphs::DMACONTROL2_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS2_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS2_CHANN_ACT,
            ),
            3 => (
                utralib::utra::udphs::DMAADDRESS3_BUFF_ADD,
                utralib::utra::udphs::DMACONTROL3_BUFF_LENGTH,
                utralib::utra::udphs::DMACONTROL3_END_BUFFIT,
                utralib::utra::udphs::DMACONTROL3_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS3_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS3_CHANN_ACT,
            ),
            _ => panic!("invalid endpoint index {}", ep_addr.index()),
        };
        udphs.wfo(f_dmaaddress, buf.as_ptr() as u32);
        udphs.wfo(f_buff_length, buf.len() as u32);
        udphs.wfo(f_end_buffit, 1);
        udphs.wfo(f_chann_enb_command, 1);
        // TODO Probably need to have a function which waits for the interrupt to send a message
        // down a channel (connection). But this condition still has to wait - at least the
        // CHANN_ACT bit has to be 0
        while udphs.rf(f_chann_enb_status) == 1 || udphs.rf(f_chann_act_status) == 1 {}
        Ok(buf.len())
    }

    // TODO This is the only thing left
    fn read(
        &self,
        ep_addr: usb_device::endpoint::EndpointAddress,
        buf: &mut [u8],
    ) -> usb_device::Result<usize> {
        let mut udphs = CSR::new(utralib::HW_UDPHS_BASE as *mut u32);
        let (
            f_dmaaddress,
            f_buff_length,
            f_end_buffit,
            f_chann_enb_command,
            f_chann_enb_status,
            f_chann_act_status,
        ) = match ep_addr.index() {
            0 => (
                utralib::utra::udphs::DMAADDRESS0_BUFF_ADD,
                utralib::utra::udphs::DMACONTROL0_BUFF_LENGTH,
                utralib::utra::udphs::DMACONTROL0_END_BUFFIT,
                utralib::utra::udphs::DMACONTROL0_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS0_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS0_CHANN_ACT,
            ),
            1 => (
                utralib::utra::udphs::DMAADDRESS1_BUFF_ADD,
                utralib::utra::udphs::DMACONTROL1_BUFF_LENGTH,
                utralib::utra::udphs::DMACONTROL1_END_BUFFIT,
                utralib::utra::udphs::DMACONTROL1_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS1_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS1_CHANN_ACT,
            ),
            2 => (
                utralib::utra::udphs::DMAADDRESS2_BUFF_ADD,
                utralib::utra::udphs::DMACONTROL2_BUFF_LENGTH,
                utralib::utra::udphs::DMACONTROL2_END_BUFFIT,
                utralib::utra::udphs::DMACONTROL2_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS2_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS2_CHANN_ACT,
            ),
            3 => (
                utralib::utra::udphs::DMAADDRESS3_BUFF_ADD,
                utralib::utra::udphs::DMACONTROL3_BUFF_LENGTH,
                utralib::utra::udphs::DMACONTROL3_END_BUFFIT,
                utralib::utra::udphs::DMACONTROL3_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS3_CHANN_ENB,
                utralib::utra::udphs::DMASTATUS3_CHANN_ACT,
            ),
            _ => panic!("invalid endpoint index {}", ep_addr.index()),
        };
        udphs.wfo(f_dmaaddress, buf.as_ptr() as u32);
        udphs.wfo(f_buff_length, buf.len() as u32);
        udphs.wfo(f_end_buffit, 1);
        udphs.wfo(f_chann_enb_command, 1);
        // TODO Probably need to have a function which waits for the interrupt to send a message
        // down a channel (connection). But this condition still has to wait - at least the
        // CHANN_ACT bit has to be 0
        while udphs.rf(f_chann_enb_status) == 1 || udphs.rf(f_chann_act_status) == 1 {}
        Ok(buf.len())
    }

    fn set_stalled(&self, ep_addr: usb_device::endpoint::EndpointAddress, stalled: bool) {
        let mut udphs = CSR::new(utralib::HW_UDPHS_BASE as *mut u32);
        udphs.wfo(
            match ep_addr.index() {
                0 => utralib::utra::udphs::EPTSETSTA0_FRCESTALL,
                1 => utralib::utra::udphs::EPTSETSTA1_FRCESTALL,
                2 => utralib::utra::udphs::EPTSETSTA2_FRCESTALL,
                3 => utralib::utra::udphs::EPTSETSTA3_FRCESTALL,
                _ => panic!("invalid endpoint index {}", ep_addr.index()),
            },
            if stalled { 1 } else { 0 },
        );
    }

    fn is_stalled(&self, ep_addr: usb_device::endpoint::EndpointAddress) -> bool {
        let mut udphs = CSR::new(utralib::HW_UDPHS_BASE as *mut u32);
        udphs.rf(match ep_addr.index() {
            0 => utralib::utra::udphs::EPTSETSTA0_FRCESTALL,
            1 => utralib::utra::udphs::EPTSETSTA1_FRCESTALL,
            2 => utralib::utra::udphs::EPTSETSTA2_FRCESTALL,
            3 => utralib::utra::udphs::EPTSETSTA3_FRCESTALL,
            _ => panic!("invalid endpoint index {}", ep_addr.index()),
        }) == 1
    }

    fn suspend(&self) {}

    fn resume(&self) {}

    fn poll(&self) -> usb_device::bus::PollResult {
        let mut udphs = CSR::new(utralib::HW_UDPHS_BASE as *mut u32);

        let mut ep_out = 0u16;
        let mut ep_in_complete = 0u16;
        let mut ep_setup = 0u16;

        if udphs.rf(utralib::utra::udphs::EPTCTLEN0_RXRDY_TXKL) == 1 {
            ep_out |= 0b0001;
        }
        if udphs.rf(utralib::utra::udphs::EPTCTLEN1_RXRDY_TXKL) == 1 {
            ep_out |= 0b0010;
        }
        if udphs.rf(utralib::utra::udphs::EPTCTLEN2_RXRDY_TXKL) == 1 {
            ep_out |= 0b0100;
        }
        if udphs.rf(utralib::utra::udphs::EPTCTLEN3_RXRDY_TXKL) == 1 {
            ep_out |= 0b1000;
        }

        if udphs.rf(utralib::utra::udphs::EPTCTLEN0_TX_COMPLT) == 1 {
            ep_in_complete |= 0b0001;
            udphs.wfo(utralib::utra::udphs::EPTRST_EPT_0, 1);
        }
        if udphs.rf(utralib::utra::udphs::EPTCTLEN1_TX_COMPLT) == 1 {
            ep_in_complete |= 0b0010;
            udphs.wfo(utralib::utra::udphs::EPTRST_EPT_1, 1);
        }
        if udphs.rf(utralib::utra::udphs::EPTCTLEN2_TX_COMPLT) == 1 {
            ep_in_complete |= 0b0100;
            udphs.wfo(utralib::utra::udphs::EPTRST_EPT_2, 1);
        }
        if udphs.rf(utralib::utra::udphs::EPTCTLEN3_TX_COMPLT) == 1 {
            ep_in_complete |= 0b1000;
            udphs.wfo(utralib::utra::udphs::EPTRST_EPT_3, 1);
        }

        if udphs.rf(utralib::utra::udphs::EPTCTLEN0_RX_SETUP) == 1 {
            ep_setup |= 0b0001;
        }
        if udphs.rf(utralib::utra::udphs::EPTCTLEN1_RX_SETUP) == 1 {
            ep_setup |= 0b0010;
        }
        if udphs.rf(utralib::utra::udphs::EPTCTLEN2_RX_SETUP) == 1 {
            ep_setup |= 0b0100;
        }
        if udphs.rf(utralib::utra::udphs::EPTCTLEN3_RX_SETUP) == 1 {
            ep_setup |= 0b1000;
        }

        if ep_out == 0 && ep_in_complete == 0 && ep_setup == 0 {
            usb_device::bus::PollResult::None
        } else {
            usb_device::bus::PollResult::Data {
                ep_out,
                ep_in_complete,
                ep_setup,
            }
        }
    }
}
