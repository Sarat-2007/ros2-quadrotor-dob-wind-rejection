#!/usr/bin/env python3
"""
create_new_template_presentation.py
New Modern Academic Presentation Template
- Sleek Deep Slate Top Banner (#1E293B) with high-contrast white headers and slide number pills.
- Clean white body background with accented card containers.
- Enforced zero-overlap geometry, large simulation figures, and 13-14pt readable typography.
- Outputs:
  1. D:/ROS/UAV_Disturbance_Observer_New_Template.pptx
  2. D:/ROS/UAV_Disturbance_Observer_Project_Presentation.pptx
"""

import os
import sys
from PIL import Image
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

IMG_DIR = "/mnt/d/ROS/docs/images"
OUT_PATHS = [
    "/mnt/d/ROS/UAV_Disturbance_Observer_New_Template.pptx",
    "/mnt/d/ROS/UAV_Disturbance_Observer_Project_Presentation.pptx"
]

# --- NEW MODERN ACADEMIC PALETTE ---
COLOR_BANNER = RGBColor(30, 41, 59)        # Deep Slate Navy (#1E293B)
COLOR_BANNER_PILL = RGBColor(51, 65, 85)   # Subdued Pill Blue/Slate (#334155)
COLOR_WHITE = RGBColor(255, 255, 255)      # Pure White
COLOR_BG = RGBColor(250, 250, 250)         # Crisp Off-White (#FAFAFA)
COLOR_TEXT_MAIN = RGBColor(30, 41, 59)     # Deep Slate Text (#1E293B)
COLOR_TEXT_BODY = RGBColor(51, 65, 85)     # Charcoal Body (#334155)
COLOR_MUTED = RGBColor(148, 163, 184)      # Slate Grey (#94A3B8)
COLOR_CARD_BG = RGBColor(255, 255, 255)    # Card White
COLOR_CARD_BORDER = RGBColor(226, 232, 240)# Soft Border (#E2E8F0)
COLOR_BLUE_ACCENT = RGBColor(37, 99, 235)  # Royal Blue Accent (#2563EB)
COLOR_GREEN_ACCENT = RGBColor(16, 185, 129)# Emerald Green Accent (#10B981)
COLOR_RED_ACCENT = RGBColor(239, 68, 68)   # Coral Red Accent (#EF4444)

def add_fitted_picture(slide, img_path, box_left, box_top, box_w, box_h):
    """Calculates aspect ratio, fits image inside bounding box without distortion, and centers it."""
    if not os.path.exists(img_path):
        return None
    with Image.open(img_path) as im:
        im_w, im_h = im.size
        im_ratio = im_w / im_h

    box_ratio = box_w / box_h

    if im_ratio > box_ratio:
        fit_w = box_w
        fit_h = box_w / im_ratio
    else:
        fit_h = box_h
        fit_w = box_h * im_ratio

    offset_x = box_left + (box_w - fit_w) / 2.0
    offset_y = box_top + (box_h - fit_h) / 2.0

    # Border frame
    border_rect = slide.shapes.add_shape(
        MSO_SHAPE.RECTANGLE, 
        Inches(offset_x - 0.04), Inches(offset_y - 0.04), 
        Inches(fit_w + 0.08), Inches(fit_h + 0.08)
    )
    border_rect.fill.solid()
    border_rect.fill.fore_color.rgb = COLOR_CARD_BG
    border_rect.line.color.rgb = COLOR_CARD_BORDER
    border_rect.line.width = Pt(1.0)

    # Insert Picture
    pic = slide.shapes.add_picture(
        img_path, 
        Inches(offset_x), Inches(offset_y), 
        Inches(fit_w), Inches(fit_h)
    )
    return pic

def create_deck():
    prs = Presentation()
    prs.slide_width = Inches(13.333)
    prs.slide_height = Inches(7.5)
    blank_layout = prs.slide_layouts[6]

    def add_base_slide(title_text, category_tag, slide_num_str):
        slide = prs.slides.add_slide(blank_layout)

        # 1. Top Header Banner (Deep Slate Navy)
        banner = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0.0), Inches(0.0), Inches(13.333), Inches(1.20))
        banner.fill.solid()
        banner.fill.fore_color.rgb = COLOR_BANNER
        banner.line.fill.background()

        # Category tag
        tag_box = slide.shapes.add_textbox(Inches(0.8), Inches(0.18), Inches(10.0), Inches(0.25))
        tf_tag = tag_box.text_frame
        tf_tag.word_wrap = True
        tf_tag.margin_left = tf_tag.margin_top = tf_tag.margin_right = tf_tag.margin_bottom = 0
        p_tag = tf_tag.paragraphs[0]
        p_tag.text = category_tag.upper()
        p_tag.font.name = "Arial"
        p_tag.font.size = Pt(9.5)
        p_tag.font.bold = True
        p_tag.font.color.rgb = RGBColor(148, 163, 184) # Light Slate

        # Slide Title
        title_box = slide.shapes.add_textbox(Inches(0.8), Inches(0.48), Inches(10.0), Inches(0.55))
        tf_title = title_box.text_frame
        tf_title.word_wrap = True
        tf_title.margin_left = tf_title.margin_top = tf_title.margin_right = tf_title.margin_bottom = 0
        p_title = tf_title.paragraphs[0]
        p_title.text = title_text
        p_title.font.name = "Arial"
        p_title.font.size = Pt(21)
        p_title.font.bold = True
        p_title.font.color.rgb = COLOR_WHITE

        # Slide Number Pill
        pill = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(11.8), Inches(0.38), Inches(0.85), Inches(0.42))
        pill.fill.solid()
        pill.fill.fore_color.rgb = COLOR_BANNER_PILL
        pill.line.fill.background()
        tf_pill = pill.text_frame
        tf_pill.margin_left = tf_pill.margin_top = tf_pill.margin_right = tf_pill.margin_bottom = 0
        p_pill = tf_pill.paragraphs[0]
        p_pill.text = slide_num_str
        p_pill.alignment = PP_ALIGN.CENTER
        p_pill.font.name = "Arial"
        p_pill.font.size = Pt(11)
        p_pill.font.bold = True
        p_pill.font.color.rgb = COLOR_WHITE

        # Footer
        footer_box = slide.shapes.add_textbox(Inches(0.8), Inches(7.12), Inches(11.733), Inches(0.25))
        tf_foot = footer_box.text_frame
        tf_foot.word_wrap = True
        tf_foot.margin_left = tf_foot.margin_top = tf_foot.margin_right = tf_foot.margin_bottom = 0
        p_foot = tf_foot.paragraphs[0]
        p_foot.text = "UAV Disturbance Observer Flight Control  |  ROS 2 Humble & Gazebo 11  |  github.com/Sarat-2007/ros2-quadrotor-dob-wind-rejection"
        p_foot.font.name = "Calibri"
        p_foot.font.size = Pt(9.0)
        p_foot.font.color.rgb = COLOR_MUTED

        return slide

    def add_card(slide, left, top, width, height, title, items, accent_color=COLOR_BLUE_ACCENT):
        # Card background
        rect = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(left), Inches(top), Inches(width), Inches(height))
        rect.fill.solid()
        rect.fill.fore_color.rgb = COLOR_CARD_BG
        rect.line.color.rgb = COLOR_CARD_BORDER
        rect.line.width = Pt(1.0)

        # Left vertical accent line
        acc = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(left), Inches(top + 0.15), Inches(0.06), Inches(height - 0.3))
        acc.fill.solid()
        acc.fill.fore_color.rgb = accent_color
        acc.line.fill.background()

        tb = slide.shapes.add_textbox(Inches(left + 0.28), Inches(top + 0.25), Inches(width - 0.5), Inches(height - 0.5))
        tf = tb.text_frame
        tf.word_wrap = True
        tf.margin_left = tf.margin_top = tf.margin_right = tf.margin_bottom = 0

        p0 = tf.paragraphs[0]
        p0.text = title
        p0.font.name = "Arial"
        p0.font.size = Pt(15)
        p0.font.bold = True
        p0.font.color.rgb = COLOR_TEXT_MAIN
        p0.space_after = Pt(12)

        for item in items:
            p = tf.add_paragraph()
            p.space_after = Pt(10)
            p.font.name = "Calibri"
            p.font.size = Pt(13)
            p.font.color.rgb = COLOR_TEXT_BODY
            if isinstance(item, tuple):
                bold_prefix, text_body = item
                r_pre = p.add_run()
                r_pre.text = bold_prefix + " "
                r_pre.font.bold = True
                r_pre.font.color.rgb = COLOR_TEXT_MAIN
                r_body = p.add_run()
                r_body.text = text_body
            else:
                r = p.add_run()
                r.text = item

    # -------------------------------------------------------------
    # SLIDE 1: HERO TITLE SLIDE (Modern Executive Slate)
    # -------------------------------------------------------------
    s1 = prs.slides.add_slide(blank_layout)

    # Top dark hero container (occupies top 56% of slide)
    hero = s1.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0.0), Inches(0.0), Inches(13.333), Inches(4.3))
    hero.fill.solid()
    hero.fill.fore_color.rgb = COLOR_BANNER
    hero.line.fill.background()

    tb_t = s1.shapes.add_textbox(Inches(1.0), Inches(0.8), Inches(11.333), Inches(3.2))
    tf_t = tb_t.text_frame
    tf_t.word_wrap = True
    tf_t.margin_left = tf_t.margin_top = tf_t.margin_right = tf_t.margin_bottom = 0

    p_badge = tf_t.paragraphs[0]
    p_badge.text = "RESEARCH & TECHNICAL PRESENTATION  |  AUTONOMOUS AERIAL ROBOTICS"
    p_badge.font.name = "Arial"
    p_badge.font.size = Pt(10.5)
    p_badge.font.bold = True
    p_badge.font.color.rgb = RGBColor(56, 189, 248) # Light Cyan Accent
    p_badge.space_after = Pt(12)

    p_title = tf_t.add_paragraph()
    p_title.text = "Nonlinear Disturbance Observer (DOB) for Autonomous Quadrotors Under Violent Wind Gusts"
    p_title.font.name = "Arial"
    p_title.font.size = Pt(26)
    p_title.font.bold = True
    p_title.font.color.rgb = COLOR_WHITE
    p_title.space_after = Pt(12)

    p_sub = tf_t.add_paragraph()
    p_sub.text = "Active Disturbance Rejection Control (ADRC), 6-DOF Rayleigh Aerodynamics, and Gazebo 11 Physics Benchmark"
    p_sub.font.name = "Calibri"
    p_sub.font.size = Pt(14)
    p_sub.font.color.rgb = RGBColor(203, 213, 225)
    p_sub.space_after = Pt(24)

    p_meta = tf_t.add_paragraph()
    p_meta.text = "Author: Nallabati Venkata Durga Sai Sarat Chandra  |  ROS 2 Humble & Gazebo 11  |  Branch: main"
    p_meta.font.name = "Calibri"
    p_meta.font.size = Pt(11)
    p_meta.font.color.rgb = RGBColor(148, 163, 184)

    # Bottom 3 Highlight Metric Cards (fills bottom 44%)
    cards_data = [
        ("85.8% DRIFT REDUCTION", "Shock gust drift cut from 86.2 cm to 12.2 cm at Inspection Gate.", COLOR_GREEN_ACCENT),
        ("35.0 rad/s OBSERVER", "Reconstructs unmodeled 11.2 N fluid drag forces in 28.5 ms.", COLOR_BLUE_ACCENT),
        ("ZERO COLLISIONS", "11-waypoint corridor fully collision-free past towers and roofs.", COLOR_BANNER)
    ]
    card_w = 3.6
    gap = 0.26
    start_x = 1.0
    for i, (head, desc, acc_col) in enumerate(cards_data):
        cx = start_x + i * (card_w + gap)
        c_box = s1.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(cx), Inches(4.7), Inches(card_w), Inches(2.2))
        c_box.fill.solid()
        c_box.fill.fore_color.rgb = COLOR_CARD_BG
        c_box.line.color.rgb = COLOR_CARD_BORDER
        c_box.line.width = Pt(1.0)

        # Accent top bar on each card
        top_bar = s1.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(cx), Inches(4.7), Inches(card_w), Inches(0.08))
        top_bar.fill.solid()
        top_bar.fill.fore_color.rgb = acc_col
        top_bar.line.fill.background()

        tb_c = s1.shapes.add_textbox(Inches(cx + 0.25), Inches(5.0), Inches(card_w - 0.5), Inches(1.7))
        tf_c = tb_c.text_frame
        tf_c.word_wrap = True
        tf_c.margin_left = tf_c.margin_top = tf_c.margin_right = tf_c.margin_bottom = 0
        p_ch = tf_c.paragraphs[0]
        p_ch.text = head
        p_ch.font.name = "Arial"
        p_ch.font.size = Pt(13)
        p_ch.font.bold = True
        p_ch.font.color.rgb = COLOR_TEXT_MAIN
        p_ch.space_after = Pt(8)

        p_cd = tf_c.add_paragraph()
        p_cd.text = desc
        p_cd.font.name = "Calibri"
        p_cd.font.size = Pt(11.5)
        p_cd.font.color.rgb = COLOR_TEXT_BODY

    # -------------------------------------------------------------
    # SLIDE 2: THE PROBLEM (PID LIMITATIONS VS ADRC GOALS)
    # -------------------------------------------------------------
    s2 = add_base_slide("Problem Statement: Aerodynamic Wind Shocks vs. Classical PID", "Control Limitation Analysis", "02 / 07")

    items_pid = [
        ("Violent Aerodynamic Drag:", "An 8.5 m/s crosswind exerts 11.2 N on a 1.52 kg drone—75% of total vehicle weight!"),
        ("Severe Reaction Delay:", "PID cannot act until large position and velocity tracking errors have already accumulated."),
        ("Dangerous Integral Windup:", "Integrator slowly winds up over seconds. When the wind stops, accumulated energy flings drone into obstacles."),
        ("Actuator Saturation & Noise:", "Pumping derivative gains amplifies IMU sensor noise, causing motor chattering and instability.")
    ]
    add_card(s2, 0.8, 1.45, 5.7, 5.4, "Conventional Autopilot Failure Modes", items_pid, COLOR_RED_ACCENT)

    items_target = [
        ("Immediate Disturbance Rejection:", "Reconstruct unmodeled fluid drag in 28.5 ms (ω_obs = 35 rad/s) before drift starts."),
        ("Feedforward Decoupling:", "Directly subtract estimated aerodynamic force in acceleration space: a_des = a_PID - (1/m) F_hat."),
        ("Nonlinear Tilt Thrust:", "Scale thrust by 1 / (cos φ · cos θ) so the drone never loses altitude when leaning up to 37°."),
        ("Strict Collision-Free Guarantee:", "Maintain ≥ 0.82 m clearance past all gates, rooftops, and towers under shock winds.")
    ]
    add_card(s2, 6.8, 1.45, 5.7, 5.4, "Our Active Disturbance Rejection Goals", items_target, COLOR_GREEN_ACCENT)

    # -------------------------------------------------------------
    # SLIDE 3: DISTURBANCE OBSERVER ARCHITECTURE & FORMULATIONS
    # -------------------------------------------------------------
    s3 = add_base_slide("Control Architecture: Two-Time-Scale Disturbance Observer", "Robust Control Theory", "03 / 07")

    items_theory = [
        ("Observer State Filter:", "dF_hat/dt = ω_obs · (F_meas - F_hat) with bandwidth ω_obs = 35.0 rad/s (τ = 28.5 ms)."),
        ("Feedforward Acceleration:", "a_des = a_ref + K_p e_p + K_i ∫e_p + K_d e_v - (1/m) F_hat. Instantly cancels drag force."),
        ("Nonlinear Tilt Compensation:", "T = (m · a_z,des) / (cos φ · cos θ). Boosts thrust from 14.9 N to 18.2 N to counter gravity during tilt."),
        ("Lyapunov Asymptotic Stability:", "Proven exponential convergence: error ||e_obs(t)|| → 0 as t → ∞ under bounded gust rates.")
    ]
    add_card(s3, 0.8, 1.45, 5.3, 5.4, "Mathematical Formulation", items_theory, COLOR_BLUE_ACCENT)

    # Fit Real Architecture Block Diagram
    add_fitted_picture(s3, os.path.join(IMG_DIR, "control_system_architecture.png"), 6.4, 1.45, 6.1, 5.4)

    # -------------------------------------------------------------
    # SLIDE 4: 3D GAZEBO SIMULATION WORLD & OBSTACLES
    # -------------------------------------------------------------
    s4 = add_base_slide("3D Simulation Arena & 11-Waypoint Autonomous Patrol Corridor", "Facility Environment Modeling", "04 / 07")

    items_arena = [
        ("6 Authentic Obstacles:", "Central Helipad (Origin), 4m Inspection Gate, 2m Elevated Rooftop Station, Towers A & B, Slalom Pylons."),
        ("Verified Safety Corridors:", "Every waypoint engineered with clearances ≥ 0.82 m - 2.10 m from prop tip to any obstacle surface."),
        ("Inspection Gate Fly-Through:", "Passes directly through gate arch (W=3.2m, H=4m) to Y=6.5m before turning, eliminating pillar collisions."),
        ("Rooftop Station Survey:", "Surveys elevated station at Z=3.5m (1.5m above roof) while actively rejecting 8.5 m/s crosswind shock.")
    ]
    add_card(s4, 0.8, 1.45, 5.2, 5.4, "Corridor Clearances", items_arena, COLOR_BLUE_ACCENT)

    # Fit Real 3D Arena Render
    add_fitted_picture(s4, os.path.join(IMG_DIR, "gazebo_3d_simulation_arena.png"), 6.3, 1.45, 6.2, 5.4)

    # -------------------------------------------------------------
    # SLIDE 5: EMPIRICAL BENCHMARK - PID VS DOB
    # -------------------------------------------------------------
    s5 = add_base_slide("Empirical Verification: Multi-Position Wind Gust Benchmark", "Quantitative Performance Results", "05 / 07")

    items_res = [
        ("Gust 1 (Gate, 7.5 m/s):", "PID Drift = 86.2 cm  →  DOB Drift = 12.2 cm\n85.8% Drift Cut! Prevents pillar collision."),
        ("Gust 2 (Rooftop, 8.5 m/s):", "PID Drift = 157.1 cm  →  DOB Drift = 41.9 cm\n73.3% Drift Cut! Prevents roof blowout."),
        ("Gust 3 (Helipad, 8.0 m/s):", "PID Drift = 84.7 cm  →  DOB Drift = 29.9 cm\n64.7% Drift Cut! Ensures 1.2m pad landing."),
        ("Overall Mean Tracking Error:", "Cut by 38.7% (20.2 cm vs. 32.9 cm across all 11 waypoints) with 0 collisions.")
    ]
    add_card(s5, 0.8, 1.45, 5.3, 5.4, "Benchmark Metrics", items_res, COLOR_GREEN_ACCENT)

    # Fit Real Multi-Gust Benchmark Figure
    add_fitted_picture(s5, os.path.join(IMG_DIR, "multi_gust_dob_vs_pid.png"), 6.4, 1.45, 6.1, 5.4)

    # -------------------------------------------------------------
    # SLIDE 6: GROUND CONTROL STATION & PILOT INTERACTION
    # -------------------------------------------------------------
    s6 = add_base_slide("Ground Control Station (GCS) & Human Teleoperation", "Industrial Pilot Dashboard", "06 / 07")

    items_gcs = [
        ("Aerospace Dark Slate HUD:", "Real-time telemetry readouts for 3D position [X,Y,Z], speed, heading, and estimated disturbance drag force."),
        ("Real-Time Aerodynamic Calculator:", "Dynamically calculates expected fluid drag Fd = ½ρCdAv², required vehicle tilt angle, and required thrust."),
        ("1-Click Autonomous Patrol Panel:", "Instantly deploys the 11-waypoint facility surveillance mission with active waypoint progression readouts."),
        ("Instant Station Hover & Override:", "Clicking 'ABORT' or touching any flight stick instantly locks drone into rock-steady zero-drift hover.")
    ]
    add_card(s6, 0.8, 1.45, 7.4, 5.4, "Operator Capabilities", items_gcs, COLOR_BLUE_ACCENT)

    # Fit Portrait GCS Dashboard HUD
    add_fitted_picture(s6, os.path.join(IMG_DIR, "gcs_teleop_dashboard.png"), 8.5, 1.45, 4.0, 5.4)

    # -------------------------------------------------------------
    # SLIDE 7: SUMMARY & FACULTY DEFENSE TAKEAWAYS
    # -------------------------------------------------------------
    s7 = add_base_slide("Project Summary & Faculty Evaluation Takeaways", "Academic Evaluation Reference", "07 / 07")

    items_sum_left = [
        ("Not a Basic Waypoint Follower:", "Standard drone projects follow waypoints in calm air. This project pioneers Active Disturbance Rejection Control (ADRC) for violent, unmodeled wind conditions."),
        ("Theoretical Rigor:", "Complete 6-DOF Newton-Euler dynamics, Rayleigh aerodynamic drag, and mathematically proven Lyapunov asymptotic stability."),
        ("Empirical Validation:", "Benchmarked in Gazebo 11 physics across 3 shock crosswinds, cutting peak drift by up to 85.8% with zero collisions.")
    ]
    add_card(s7, 0.8, 1.45, 5.7, 5.4, "Why This Project Stands Out", items_sum_left, COLOR_BLUE_ACCENT)

    items_sum_right = [
        ("Modular ROS 2 Architecture:", "Clean decoupling across uav_control (100 Hz), uav_aerodynamics, uav_dashboard, and Gazebo physics bridge."),
        ("Reproducible 1-Click Suite:", "Automated batch/shell scripts allow instant live verification of patrol and gust benchmarks."),
        ("Hardware-Ready Formulation:", "Differential flatness and acceleration decoupling structure maps directly to PX4 / Pixhawk autopilot hardware.")
    ]
    add_card(s7, 6.8, 1.45, 5.7, 5.4, "Engineering Deliverables", items_sum_right, COLOR_GREEN_ACCENT)

    for out_p in OUT_PATHS:
        try:
            prs.save(out_p)
            print(f"Presentation saved successfully to: {out_p}")
        except Exception as e:
            print(f"Notice: Could not save to {out_p}: {e}")

if __name__ == "__main__":
    create_deck()
