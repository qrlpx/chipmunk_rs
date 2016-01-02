/* NOTE: OUTDATED CODE
 * TODO: update
#![feature(box_syntax)]

extern crate gl;
extern crate glutin;

extern crate shader_version;
extern crate graphics;
extern crate opengl_graphics;
extern crate vecmath;

extern crate clock_ticks;
extern crate "chipmunk" as cp;
extern crate "nalgebra" as na;

use graphics::BackEnd;
use na::Zero;

use std::time::Duration;

mod math {
    pub use graphics::vecmath::*;
    pub use vecmath::*;
}

fn main(){
    let mut space = cp::Space::new();
    space.set_gravity(na::Vec2::new(0.0, -8.0));
    space.set_sleep_time_treshold(0.5);

    let ground = space.add_body(box cp::Body::new_static());
    space.attach_shape(
        ground, box cp::SegmentShape::new(
            na::Pnt2::new(-30.0, -20.0), na::Pnt2::new(30.0, -20.0), 0.05
        )
    );
    space.attach_shape(
        ground, box cp::SegmentShape::new(
            na::Pnt2::new(30.0, -20.0), na::Pnt2::new(30.0, 20.0), 0.05
        )
    );
    space.attach_shape(
        ground, box cp::SegmentShape::new(
            na::Pnt2::new(30.0, 20.0), na::Pnt2::new(-30.0, 20.0), 0.05
        )
    );
    space.attach_shape(
        ground, box cp::SegmentShape::new(
            na::Pnt2::new(-30.0, 20.0), na::Pnt2::new(-30.0, -20.0), 0.05
        )
    );

    let mut window = glutin::WindowBuilder::new()
        .with_title("chipmunk_rs - draw test".to_string())
        .with_dimensions(1200, 800)
        .with_multisampling(4)
        .with_gl_version((3, 2))
        .build().unwrap();
    unsafe {
        window.make_current(); 
        gl::load_with(|sym| window.get_proc_address(sym));
    }

    let mut gl = opengl_graphics::Gl::new(shader_version::OpenGL::_3_2);
    let context = graphics::Context{
        view: math::translate([60.0, 40.0]),
        transform: math::scale(0.032, 0.048),
    };

    let active_color = [0.7, 0.7, 0.7, 1.0];
    let inactive_color = [0.3, 0.3, 0.3, 1.0];
    
    let active_circle = graphics::Ellipse::new(active_color);
    let inactive_circle = graphics::Ellipse::new(inactive_color);

    let mut mouse = na::orig::<cp::Pnt2>();

    let mut body_count = 0us;
    let mut last_time = clock_ticks::precise_time_s();
    let mut second_remaining = 1.0;
    let mut current_ticks = 0;

    while !window.is_closed() {
        for ev in window.poll_events() {
            match ev {
                glutin::Event::MouseMoved((x, y)) => {
                    mouse.x = (x as cp::float - 600.0) * 0.048;
                    mouse.y = -(y as cp::float - 400.0) * 0.032;
                }
                glutin::Event::MouseInput(
                    glutin::ElementState::Pressed,
                    glutin::MouseButton::LeftMouseButton
                ) => {
                    let mut body = cp::Body::new(1.0, 1.0);
                    body.set_position(mouse);
                    let body = space.add_body(box body);
                    body_count += 1;
 
                    let mut circle = cp::CircleShape::new(0.5, None);
                    circle.set_mass(2.6);
                    circle.set_elasticity(0.8);
                    circle.set_density(0.5);
                    circle.set_friction(10.0);
                    space.attach_shape(body, box circle);
                }
                _ => {}
            }
        }
        gl.clear([0.9, 0.9, 0.9, 1.0]);

        for shape in space.iter_shapes() {
            match shape.as_circle_ref(){
                Some(circle) => {
                    let pos = circle.body().position();
                    let d = circle.radius() * 2.0;
                    if circle.body().is_sleeping() {
                        inactive_circle
                    } else {
                        active_circle
                    }.draw(
                        [pos.x as f64, pos.y as f64, d as f64, d as f64], 
                        &context, &mut gl
                    );
                },
                None => {}
            }
        }

        let dt = clock_ticks::precise_time_s() - last_time;
        last_time += dt;
        current_ticks += 1;
        second_remaining -= dt;
        if second_remaining < 0.0 {
            second_remaining = 1.0;
            println!("fps: {}; bodies: {}", current_ticks, body_count);
            current_ticks = 0;
        } 
        space.step(dt as f32);
        window.swap_buffers();
    }
}
*/
