
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Raw3 {
    x: i16,
    y: i16,
    z: i16,
}

impl Raw3 {
    pub fn new(x: i16, y: i16, z: i16) -> Self {
        Self {
            x,
            y,
            z,
        }
    }

    pub fn x(self) -> i16 {
        self.x
    }

    pub fn y(self) -> i16 {
        self.y
    }

    pub fn z(self) -> i16 {
        self.z
    }
}