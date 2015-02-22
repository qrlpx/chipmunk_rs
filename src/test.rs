use na;

use super::{Body};


#[test]
fn f32_f64_linkage_error(){
    let mut space = super::Space::new();
    unsafe {
        assert_eq!(space.mut_ground_body().position().x, (*(*space.ptr()).staticBody).p.x);
    }
}

