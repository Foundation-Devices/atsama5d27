use embedded_graphics::{
    pixelcolor::{raw::RawU24, Rgb888},
    prelude::*,
    primitives::Rectangle,
};

pub struct FramebufDisplay {
    fb: &'static mut [u32],
    w: usize,
    h: usize,
}

impl FramebufDisplay {
    pub fn new(fb: &'static mut [u32], w: usize, h: usize) -> FramebufDisplay {
        FramebufDisplay { fb, w, h }
    }

    pub fn scroll_up(&mut self, amount: usize) {
        let start = amount * self.w;
        self.fb.copy_within(start.., 0);

        let last_line = self.w * self.h - start;
        self.fb[last_line..].fill(RawU24::from(Rgb888::BLACK).into_inner());
    }

    pub fn width(&self) -> usize {
        self.w
    }

    pub fn height(&self) -> usize {
        self.h
    }
}

impl Dimensions for FramebufDisplay {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::with_corners(Point::new(0, 0), Point::new(self.w as i32, self.h as i32))
    }
}

impl DrawTarget for FramebufDisplay {
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
            self.fb[self.w * y + x] = RawU24::from(color).into_inner();
        }

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let color = RawU24::from(color).into_inner();
        self.fb.fill(color);
        Ok(())
    }
}
