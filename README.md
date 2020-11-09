# AUGMENTED-REALITY-APRILTAG

This package provides the functionality of rendering a 3D model on a detected Apriltag

## Instructions

### 1) Clone the repository

git clone https://github.com/alvarobelmontebaeza/augmented-reality-apriltag

### 2) Move into the root diretory of the repository

### 2) Build the package in your Duckiebot

dts devel build -f -H  HOSTNAME.local

### 4) Run the node

dts devel run -H HOSTNAME.local

### 5) In another terminal, open rqt_image_view and select the correct topic to visualize the AR image

dts start_gui_tools HOSTNAME

rqt_image_view

### 6) Enjoy!
