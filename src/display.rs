use embedded_graphics::{
    pixelcolor::{raw::RawU24, Rgb888},
    prelude::*,
    primitives::Rectangle,
};
use crate::lcdc::{Lcdc, LcdcLayerId, LcdDmaDesc};

pub struct DoubleBufferedDisplay<'a> {
    lcdc: Lcdc,
    fb1: &'a mut [u32],
    fb2: &'a mut [u32],
    dma1: usize,
    dma2: usize,
    w: usize,
    h: usize,
}

impl<'a> DoubleBufferedDisplay<'a> {
    pub fn new(lcdc: Lcdc, fb1: &'a mut [u32], fb2: &'a mut [u32], dma1: usize, dma2: usize, w: usize, h: usize) -> DoubleBufferedDisplay<'a> {
        DoubleBufferedDisplay { lcdc, fb1, fb2, dma1, dma2, w, h }
    }

    pub fn scroll_up(&mut self, amount: usize) {
        let start = amount * self.w;
        self.fb1.copy_within(start.., 0);

        let last_line = self.w * self.h - start;
        self.fb1[last_line..].fill(RawU24::from(Rgb888::BLACK).into_inner());
    }

    pub fn width(&self) -> usize {
        self.w
    }

    pub fn height(&self) -> usize {
        self.h
    }
}

impl Dimensions for DoubleBufferedDisplay<'_> {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::with_corners(Point::new(0, 0), Point::new(self.w as i32, self.h as i32))
    }
}

impl DrawTarget for DoubleBufferedDisplay<'_> {
    type Color = Rgb888;
    type Error = ();

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels.into_iter() {
            if point.x < 0
                || point.y < 0
                || point.x as usize >= self.w
                || point.y as usize >= self.h
            {
                continue;
            }

            let x = point.x as usize;
            let y = point.y as usize;
            self.fb1[self.w * y + x] = RawU24::from(color).into_inner();
        }

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let color = RawU24::from(color).into_inner();
        self.fb1.fill(color);
        Ok(())
    }
}

impl<'a> DoubleBufferedDisplay<'a> {
    /// Swaps two buffers to apply drawing changes.
    pub fn flush(&mut self) {
        /*
        let len = self.fb1.len();
        let fb1 = self.fb1.as_mut_ptr();
        let fb2 = self.fb2.as_mut_ptr();
        unsafe {
            self.fb1 = core::slice::from_raw_parts_mut(fb2, len);
            self.fb2 = core::slice::from_raw_parts_mut(fb1, len);
        }*/

        /*let swap = self.dma2;
        self.dma2 = self.dma1;
        self.dma1 = swap;*/

        let dma_addr = self.dma1;
        let dma_desc = dma_addr as *mut LcdDmaDesc;
        unsafe {
            (*dma_desc).addr = self.fb1.as_ptr() as u32;
            (*dma_desc).ctrl = 0x01;
            (*dma_desc).next = dma_addr as u32;
        }

        self.lcdc.update_buffer(LcdcLayerId::Base,  dma_addr as u32);
    }
}
