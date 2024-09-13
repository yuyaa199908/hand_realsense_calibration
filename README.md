# hand_realsense_calibration
hoge
## param
## launch
## note
- input: raw_image, tf (map -> hand), key action
- process:
    - get image and tf (map -> hand)
    - calibration hand -> camera
    - create board mesh for visualization
    - save data (map -> hand, board -> camera)
- output: None
