extern crate console_error_panic_hook;

use std::cell::Cell;
use std::cell::RefCell;
use std::panic;
use std::rc::Rc;
use trajectory_planner::{Limits, TrajectorySegment};
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::Element;
use web_sys::Node;

macro_rules! log {
    ( $( $t:tt )* ) => {
        web_sys::console::log_1(&format!( $( $t )* ).into());
    }
}

struct Controls {
    max_velocity: Element,
}

fn display_config(container: &Element, config: &TrajectorySegment) {
    container.set_inner_html(&format!("{:#?}", config));
}

#[wasm_bindgen]
pub fn start(container: web_sys::HtmlDivElement) -> Result<(), JsValue> {
    panic::set_hook(Box::new(console_error_panic_hook::hook));

    log!("Thing: {:#?}", container);

    let document = web_sys::window().unwrap().document().unwrap();
    let canvas = document
        .create_element("canvas")?
        .dyn_into::<web_sys::HtmlCanvasElement>()?;

    container.prepend_with_node_1(&canvas)?;

    let control_inputs = container
        .query_selector(".demo-controls")?
        .expect("Element .demo-controls does not exist");

    let out = container
        .query_selector(".out")?
        .expect("Element .out is missing");

    let controls = Controls {
        max_velocity: control_inputs
            .query_selector("[name=max_velocity]")?
            .expect("Required input name max_velocity missing"),
    };

    let width = 640;
    let height = 480;

    canvas.set_width(width);
    canvas.set_height(height);
    canvas.style().set_property("border", "solid")?;
    canvas.style().set_property("border-width", "1px")?;

    let context = canvas
        .get_context("2d")?
        .unwrap()
        .dyn_into::<web_sys::CanvasRenderingContext2d>()?;

    let num_steps = 100;
    let padding_left = 10.0;

    let segment = TrajectorySegment::new(
        0.0,
        10.0,
        0.0,
        0.0,
        Limits {
            velocity: 2.0,
            acceleration: 5.0,
        },
    );

    // web_sys::console::log_1(&format!("Traj: {:#?}", segment).into());

    // Position
    context.begin_path();
    context.move_to(padding_left, (height / 2) as f64);
    context.set_stroke_style(&("#000".into()));

    for i in 0..num_steps {
        let time = segment.duration() * i as f32 / num_steps as f32;

        let y = (height / 2) as f32 - segment.position(time) * 10.0;

        // web_sys::console::log_1(&format!("Pos T: {}, Y: {}", time, y).into());

        context.line_to(
            padding_left + (width / num_steps) as f64 * i as f64,
            y as f64,
        );
    }

    context.stroke();
    context.close_path();

    // Velocity
    context.begin_path();
    context.move_to(padding_left, (height / 2) as f64);
    context.set_stroke_style(&("#f00".into()));

    for i in 0..num_steps {
        let time = segment.duration() * i as f32 / num_steps as f32;

        let y = (height / 2) as f32 - segment.velocity(time) * 10.0;

        // web_sys::console::log_1(&format!("Vel T: {}, Y: {}", time, y).into());

        context.line_to(
            padding_left + (width / num_steps) as f64 * i as f64,
            y as f64,
        );
    }

    context.stroke();
    context.close_path();

    // Acceleration
    context.begin_path();
    context.move_to(padding_left, (height / 2) as f64);
    context.set_stroke_style(&("#00f".into()));

    for i in 0..num_steps {
        let time = segment.duration() * i as f32 / num_steps as f32;

        let y = (height / 2) as f32 - segment.acceleration(time) * 10.0;

        // web_sys::console::log_1(&format!("Vel T: {}, Y: {}", time, y).into());

        context.line_to(
            padding_left + (width / num_steps) as f64 * i as f64,
            y as f64,
        );
    }

    context.stroke();
    context.close_path();

    // let context = Rc::new(context);
    // let pressed = Rc::new(Cell::new(false));

    let controls = Rc::new(controls);
    let out = Rc::new(out);
    let segment = Rc::new(RefCell::new(segment));

    {
        let controls = controls.clone();
        let out = out.clone();
        let segment = segment.clone();

        let closure = Closure::wrap(Box::new(move |event: web_sys::InputEvent| {
            let value = event
                .target()
                .as_ref()
                .map(|t| wasm_bindgen::JsCast::dyn_ref::<web_sys::HtmlInputElement>(t))
                .expect("Unable to get value")
                .expect("Unable to get value")
                .value();

            let max_velocity = value.parse::<f32>().expect("Value is not valid f32");

            segment.borrow_mut().set_velocity_limit(max_velocity);

            display_config(&out, &segment.borrow());
        }) as Box<dyn FnMut(_)>);

        controls
            .max_velocity
            .add_event_listener_with_callback("input", closure.as_ref().unchecked_ref())?;
        closure.forget();
    }
    // {
    //     let context = context.clone();
    //     let pressed = pressed.clone();
    //     let closure = Closure::wrap(Box::new(move |event: web_sys::MouseEvent| {
    //         if pressed.get() {
    //             context.line_to(event.offset_x() as f64, event.offset_y() as f64);
    //             context.stroke();
    //             context.begin_path();
    //             context.move_to(event.offset_x() as f64, event.offset_y() as f64);
    //         }
    //     }) as Box<dyn FnMut(_)>);
    //     canvas.add_event_listener_with_callback("mousemove", closure.as_ref().unchecked_ref())?;
    //     closure.forget();
    // }
    // {
    //     let context = context.clone();
    //     let pressed = pressed.clone();
    //     let closure = Closure::wrap(Box::new(move |event: web_sys::MouseEvent| {
    //         pressed.set(false);
    //         context.line_to(event.offset_x() as f64, event.offset_y() as f64);
    //         context.stroke();
    //     }) as Box<dyn FnMut(_)>);
    //     canvas.add_event_listener_with_callback("mouseup", closure.as_ref().unchecked_ref())?;
    //     closure.forget();
    // }

    Ok(())
}
