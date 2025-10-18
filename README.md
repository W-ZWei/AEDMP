
## Installation
1. Our code runs on the Ubuntu 20.04 system, with the corresponding version of ROS (ros-noetic) installed, along with the rviz and Gazebo environments.

2. Create conda environment and install all dependencies:

    conda create -n aedmp python=3.8 anaconda
    conda activate aedmp

    # note: PyTorch 2.0 works as well, 
    conda install pytorch=1.13.0 torchvision=0.14.0 torchaudio pytorch-cuda=11.7 -c pytorch -c nvidia
        Step 1: Check Your Computer's CUDA Version
            This is the most critical step, as it determines which version of PyTorch you should install.
            Open "Command Prompt" or "PowerShell".
            Enter the following command and press Enter:
            bash
            nvidia-smi  
            Look for the CUDA Version field in the top-right corner of the output.
            For example, if it shows CUDA Version: 12.4, your driver supports up to CUDA 12.4 runtime.
            PyTorch is generally backward compatible, so you can install versions built for CUDA 11.8, CUDA 12.1, etc.
            General recommendation:
            If your CUDA Version is 11.x, install a cu11x version of PyTorch.
            If it is 12.x, install a cu12x version for the best compatibility.
            Note: nvidia-smi shows the maximum CUDA version supported by your NVIDIA driver, not the CUDA toolkit version installed on your system. For installing PyTorch, knowing this maximum supported version is sufficient.

        Step 2: Visit the PyTorch Official Website for Installation Commands
            The PyTorch website provides a convenient configuration tool that automatically generates the correct installation command.

            Open the PyTorch installation guide page:
            https://pytorch.org/get-started/locally/
            Configure the options based on your needs (common example below):
            Package: Recommend pip (more universal) or conda (if using Anaconda).
            Language: Choose "Python".
            Compute Platform: This is the core option! Select based on the CUDA version from Step 1.
            If you support CUDA 12.x, choose CUDA 12.1.
            If you support CUDA 11.x, choose CUDA 11.8.
            If your computer has no NVIDIA GPU, choose CPU.
            After selecting, the website will generate the corresponding installation command.


    # remaining pip dependencies
    python -m pip install 'opencv-python>=4.2.0.34'    # newer versions may work as well
    python -m pip install torchmetrics==0.10.2
    python -m pip install wandb==0.13.6

    # optional dependencies
    # -> test dependencies and ./external only
    conda install 'protobuf<=3.19.1'    # for onnx
    python -m pip install onnx==1.13.1
    python -m pip install git+https://github.com/cocodataset/panopticapi.git
    # -> for ./external only
    # see: https://detectron2.readthedocs.io/en/latest/tutorials/install.html
    python -m pip install 'git+https://github.com/facebookresearch/detectron2.git'
    ```

3. Install submodule packages:
    ```bash
    # dataset package
    python -m pip install -e ./lib/nicr-scene-analysis-datasets[withpreparation]

    # multitask scene analysis package
    python -m pip install -e ./lib/nicr-multitask-scene-analysis
    ```
4. Prepare datasets:  
    We trained our networks on 
    [NYUv2](https://cs.nyu.edu/~silberman/datasets/nyu_depth_v2.html), 
    [SUNRGB-D](https://rgbd.cs.princeton.edu/), and 
    [Hypersim](https://machinelearning.apple.com/research/hypersim). 


We provide scripts for inference on both samples drawn from one of our used 
datasets (`main/main.py` with additional arguments) and samples located in 
`./samples` (`inference_samples.py`). 

> Note that building the model correctly depends on the respective dataset the 
model was trained on.

To run inference on a dataset with the full multi-task approach, use `main.py` 
together with `--validation-only` and `--visualize-validation`.
By default the visualized outputs are written to a newly created directory next 
to the weights. However, you can also specify the output path with 
`--visualization-output-path`.

> Note, for newer versions of TensorRT `onnx2trt` is not required (and also
not available) anymore. Pass `--trt-use-get-engine-v2` to 
`inference_time_whole_model.py` to use TensoRT's Python API instead.

We timed the inference on an NVIDIA Jetson AGX Xavier with Jetpack 4.6 
(TensorRT 8.0.1.6, PyTorch 1.10.0).

Reproducing the timings on an NVIDIA Jetson AGX Xavier further requires:
- [the PyTorch 1.10.0 wheel](https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl) from [NVIDIA Forum](https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-10-now-available/72048) (see the instruction to install TorchVision 0.11.1 as well)
- [the NVIDIA TensorRT Open Source Software](https://github.com/NVIDIA/TensorRT/tree/8.0.1) (`onnx2trt` is used to convert the onnx model to a TensorRT engine) 
- the requirements and instructions listed below:
    ```bash
    # do not use numpy>=1.19.4 or add to .bashrc "export OPENBLAS_CORETYPE=ARMV8"
    pip3 install -U numpy<=1.19

    # pycuda
    sudo ln -s /usr/include/locale.h /usr/include/xlocale.h
    pip3 install pycuda>=2021.1

    # remaining dependencies
    pip3 install dataclasses==0.8
    pip3 install protobuf==3.19.3
    pip3 install termcolor==1.1.0
    pip3 install 'tqdm>=4.62.3'
    pip3 install torchmetrics==0.6.2

    # for visualization to fix "ImportError: The _imagingft C module is not installed"
    sudo apt-get install libfreetype6-dev
    pip3 uninstall pillow
    pip3 install --no-cache-dir pillow

    # packages included as submodules in this repository
    pip install -e ./lib/nicr-scene-analysis-datasets
    pip install -e ./lib/nicr-multitask-scene-analysis
    ```

Subsequently, you can run `inference_time.bash` to reproduce the reported timings.

3. running

Run commands in the ROS environment:
```bash
rosrun pointcloud_pkg pointcloud_generation
```
Run the command in the created virtual environment:
```bash
cd main
python inference_samples1.py --dataset nyuv2 --tasks semantic instance --enable-panoptic --rgb-encoder-backbone resnet34 --rgb-encoder-backbone-block nonbottleneck1d --depth-encoder-backbone resnet34 --depth-encoder-backbone-block nonbottleneck1d --no-pretrained-backbone --input-modalities rgb depth --raw-depth --depth-max 10000 --depth-scale 1 --instance-offset-distance-threshold 40 --weights-filepath ./trained_models/skpt_resume.pth
```

Run commands in the ROS environment:
```bash
roslaunch tiago_2dnav_gazebo tiago:=true world:=222
```

4. Acknowledgements

Thanks to the work EMSANet: Efficient Multi-Task RGB-D Scene Analysis for Indoor Environments, EMSANet provides a strong foundation for the implementation of our work.


