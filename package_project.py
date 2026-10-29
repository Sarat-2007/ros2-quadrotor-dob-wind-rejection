#!/usr/bin/env python3
import os
import shutil
import zipfile

BASE_DIR = "/mnt/d/ROS"
PKG_DIR = os.path.join(BASE_DIR, "UAV_Disturbance_Observer_Project_Package")
ZIP_FILE = os.path.join(BASE_DIR, "UAV_Disturbance_Observer_Project_Package.zip")

print("Cleaning previous package directory and zip...")
if os.path.exists(PKG_DIR):
    shutil.rmtree(PKG_DIR)
if os.path.exists(ZIP_FILE):
    os.remove(ZIP_FILE)

os.makedirs(PKG_DIR, exist_ok=True)

# 1. Reports directory
reports_dir = os.path.join(PKG_DIR, "reports")
os.makedirs(reports_dir, exist_ok=True)
report_files = [
    "UAV_Disturbance_Observer_Project_Report.docx",
    "UAV_Disturbance_Observer_Project_Report.pdf",
    "UAV_Disturbance_Observer_Presentation.pptx",
    "academic_project_report.md",
    "COMPLETE_PROJECT_DOCUMENTATION.md",
    "USER_GUIDE.md"
]
for rf in report_files:
    src_f = os.path.join(BASE_DIR, rf)
    if os.path.exists(src_f):
        shutil.copy2(src_f, reports_dir)

# Also copy docx, pdf and pptx to root of package for quick access
shutil.copy2(os.path.join(BASE_DIR, "UAV_Disturbance_Observer_Project_Report.docx"), PKG_DIR)
shutil.copy2(os.path.join(BASE_DIR, "UAV_Disturbance_Observer_Project_Report.pdf"), PKG_DIR)
if os.path.exists(os.path.join(BASE_DIR, "UAV_Disturbance_Observer_Presentation.pptx")):
    shutil.copy2(os.path.join(BASE_DIR, "UAV_Disturbance_Observer_Presentation.pptx"), PKG_DIR)
shutil.copy2(os.path.join(BASE_DIR, "README.md"), PKG_DIR)
if os.path.exists(os.path.join(BASE_DIR, "LICENSE")):
    shutil.copy2(os.path.join(BASE_DIR, "LICENSE"), PKG_DIR)

# 2. Simulation Images directory
img_dest = os.path.join(PKG_DIR, "simulation_images")
os.makedirs(img_dest, exist_ok=True)
src_img_dir = os.path.join(BASE_DIR, "docs", "images")
if os.path.exists(src_img_dir):
    for f in os.listdir(src_img_dir):
        shutil.copy2(os.path.join(src_img_dir, f), img_dest)

# 3. Benchmark Data
data_dest = os.path.join(PKG_DIR, "benchmark_data")
os.makedirs(data_dest, exist_ok=True)
data_files = [
    "multi_gust_benchmark_pid.csv",
    "multi_gust_benchmark_dob.csv",
    "experiment_flight_log.csv"
]
for df in data_files:
    src_f = os.path.join(BASE_DIR, df)
    if os.path.exists(src_f):
        shutil.copy2(src_f, data_dest)

# 4. Source code
src_dest = os.path.join(PKG_DIR, "src")
shutil.copytree(os.path.join(BASE_DIR, "src"), src_dest, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))

# 5. Root scripts and launchers
for f in os.listdir(BASE_DIR):
    full_path = os.path.join(BASE_DIR, f)
    if os.path.isfile(full_path):
        if f.endswith(('.py', '.bat', '.sh')) and f != "package_project.py":
            shutil.copy2(full_path, PKG_DIR)

print(f"Package directory populated at: {PKG_DIR}")

# 6. Compress to ZIP archive
print("Compressing package into zip archive...")
with zipfile.ZipFile(ZIP_FILE, 'w', zipfile.ZIP_DEFLATED) as zf:
    for root, dirs, files in os.walk(PKG_DIR):
        for file in files:
            file_path = os.path.join(root, file)
            arcname = os.path.relpath(file_path, PKG_DIR)
            zf.write(file_path, arcname)

size_mb = os.path.getsize(ZIP_FILE) / (1024 * 1024)
print(f"ZIP Archive successfully created!")
print(f"File: {ZIP_FILE}")
print(f"Size: {size_mb:.2f} MB")
