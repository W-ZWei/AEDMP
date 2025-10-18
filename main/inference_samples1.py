# -*- coding: utf-8 -*-
"""
.. codeauthor:: Mona Koehler <mona.koehler@tu-ilmenau.de>
.. codeauthor:: Daniel Seichter <daniel.seichter@tu-ilmenau.de>
"""
from glob import glob
import os

import mmap
import cv2
import matplotlib.pyplot as plt
import torch
import numpy as np

from nicr_mt_scene_analysis.data import move_batch_to_device
from nicr_mt_scene_analysis.data import mt_collate

from emsanet.args import ArgParserEMSANet
from emsanet.data import get_datahelper
from emsanet.model import EMSANet
from emsanet.preprocessing import get_preprocessor
from emsanet.visualization import visualize_predictions
from emsanet.weights import load_weights
import time
from PIL.Image import Image


def _get_args():
    parser = ArgParserEMSANet()

    # add additional arguments
    group = parser.add_argument_group('Inference')
    group.add_argument(    # useful for appm context module
        '--inference-input-height',
        type=int,
        default=480,
        dest='validation_input_height',    # used in test phase
        help="Network input height for predicting on inference data."
    )
    group.add_argument(    # useful for appm context module
        '--inference-input-width',
        type=int,
        default=640,
        dest='validation_input_width',    # used in test phase
        help="Network input width for predicting on inference data."
    )
    group.add_argument(
        '--depth-max',
        type=float,
        default=None,
        help="Additional max depth values. Values above are set to zero as "
             "they are most likely not valid. Note, this clipping is applied "
             "before scaling the depth values."
    )
    group.add_argument(
        '--depth-scale',
        type=float,
        default=1.0,
        help="Additional depth scaling factor to apply."
    )

    return parser.parse_args()


def _load_img(img):
    #img = cv2.imread(fp, cv2.IMREAD_UNCHANGED)
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img


def main():
    
    args = _get_args()
    assert all(x in args.input_modalities for x in ('rgb', 'depth')), \
        "Only RGBD inference supported so far"

    device = torch.device('cuda')
    # data and model
    data = get_datahelper(args)
    dataset_config = data.dataset_config
    model = EMSANet(args, dataset_config=dataset_config)

    # load weights
    print(f"Loading checkpoint: '{args.weights_filepath}'")
    checkpoint = torch.load(args.weights_filepath)
    #checkpoint = torch.load(args.weights_filepath, map_location=torch.device('cpu'))
    state_dict = checkpoint['state_dict']
    if 'epoch' in checkpoint:
        print(f"-> Epoch: {checkpoint['epoch']}")
    load_weights(args, model, state_dict, verbose=True)
    torch.set_grad_enabled(False)
    model.eval()
    model.to(device)

    # build preprocessor
    preprocessor = get_preprocessor(
        args,
        dataset=data.datasets_valid[0],
        phase='test',
        multiscale_downscales=None
    )
    while True:
        try:
            with open("/dev/shm/image_data", "r+b") as r: 

                mmapped_rgb = mmap.mmap(r.fileno(), 0) 

                rows, cols, channels = 480, 640, 3

                rgb_array = np.frombuffer(mmapped_rgb, dtype=np.uint8).reshape((rows, cols, channels))

            with open("/dev/shm/depth_data", "r+b") as d: 

                mmapped_depth = mmap.mmap(d.fileno(), 0) 

                depth_array = np.frombuffer(mmapped_depth, dtype=np.uint16).reshape((rows, cols))

        except FileNotFoundError:
            #print("No image data, Waiting...")
            time.sleep(0.01)
        except ValueError:
            time.sleep(0.01)

        else:
            
            os.unlink("/dev/shm/image_data") 
            os.unlink("/dev/shm/depth_data")

            img_rgb = _load_img(rgb_array)

            img_depth = _load_img(depth_array).astype('float32')
            if args.depth_max is not None:
                img_depth[img_depth > args.depth_max] = 0
            img_depth *= args.depth_scale

            # preprocess sample
            
            sample = preprocessor({
                'rgb': img_rgb,
                'depth': img_depth,
                'identifier': "sample"
            })

                # add batch axis as there is no dataloader
            
            batch = mt_collate([sample])
            batch = move_batch_to_device(batch, device=device)

            # apply model
            start_time = time.perf_counter()
            predictions = model(batch, do_postprocessing=True)
            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            print(elapsed_time)
            #print(type(predictions['panoptic_segmentation_deeplab_instance_idx_fullres']))
            #print(predictions['panoptic_segmentation_deeplab_instance_idx_fullres'].shape)
            #print(predictions['panoptic_segmentation_deeplab_instance_idx_fullres'].cpu().numpy().shape)
            id = np.unique(predictions['panoptic_segmentation_deeplab_instance_idx_fullres'][0].cpu().numpy())
            id1 = np.unique(predictions['panoptic_segmentation_deeplab_semantic_idx_fullres'][0].cpu().numpy())
            print(id1)
            num = len(id)
            reshape_prediction_s = predictions['panoptic_segmentation_deeplab_semantic_idx_fullres'][0].cpu().numpy().astype(np.uint8)
            reshape_prediction_i = predictions['panoptic_segmentation_deeplab_instance_idx_fullres'][0].cpu().numpy()
            reshape_prediction = np.stack((reshape_prediction_i, reshape_prediction_s), axis = -1)
            print(reshape_prediction.shape)
            # visualize predictions
            #start_time = time.perf_counter()
            preds_viz = visualize_predictions(
                predictions=predictions,
                batch=batch,
                dataset_config=dataset_config
            )
            preds_viz['panoptic_segmentation_deeplab_semantic_idx_fullres'][0].save('image.png')
            preds_viz['panoptic_segmentation_deeplab_instance_idx_fullres'][0].save('image1.png')
            #end_time = time.perf_counter()
            #elapsed_time = end_time - start_time
            #print(elapsed_time)
            #print(type(preds_viz['panoptic_segmentation_deeplab_instance_idx_fullres'][0]))
            #print(preds_viz['panoptic_segmentation_deeplab_instance_idx_fullres'][0].shape)
            start_time = time.perf_counter()
            with open("/dev/shm/seg_data", "w+b") as file:
                file.truncate(reshape_prediction.nbytes)
                with mmap.mmap(file.fileno(), 0, access = mmap.ACCESS_WRITE) as shared_memory:
                    shared_memory.write(reshape_prediction.tobytes())
            with open("/dev/shm/seg_done", "w+b") as file_sym:
                file_sym.truncate(id.nbytes)
                with mmap.mmap(file_sym.fileno(), 0, access = mmap.ACCESS_WRITE) as a:
                    a.write(num.to_bytes(1, byteorder="little"))

            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            print(elapsed_time)
    #plt.imshow(preds_viz['panoptic_segmentation_deeplab_instance_idx_fullres'][0])
    #time.sleep(10)

    '''# show results
    _, axs = plt.subplots(2, 4, figsize=(12, 6), dpi=150)
    [ax.set_axis_off() for ax in axs.ravel()]

    axs[0, 0].set_title('RGB')
    axs[0, 0].imshow(
        img_rgb
    )
    axs[0, 1].set_title('Depth')
    axs[0, 1].imshow(
        img_depth,
        interpolation='nearest'
    )
    axs[0, 2].set_title('Semantic')
    axs[0, 2].imshow(
        preds_viz['semantic_segmentation_idx_fullres'][0],
        interpolation='nearest'
    )
    axs[0, 3].set_title('Semantic (panoptic)')
    axs[0, 3].imshow(
        preds_viz['panoptic_segmentation_deeplab_semantic_idx_fullres'][0],
        interpolation='nearest'
    )
    axs[1, 0].set_title('Instance (panoptic)')
    axs[1, 0].imshow(
        preds_viz['panoptic_segmentation_deeplab_instance_idx_fullres'][0],
        interpolation='nearest'
    )
    axs[1, 1].set_title('Instance centers')
    axs[1, 1].imshow(
        preds_viz['instance_centers'][0]
    )
    axs[1, 2].set_title('Instance offsets')
    axs[1, 2].imshow(
        preds_viz['instance_offsets'][0]
    )
    axs[1, 3].set_title('Panoptic (with orientations)')
    axs[1, 3].imshow(
        preds_viz['panoptic_orientations_fullres'][0],
        interpolation='nearest'
    )

    plt.suptitle("sample")
    plt.tight_layout()

    # fp = os.path.join('./', 'samples', f'result_{args.dataset}.png')
    # plt.savefig(fp, bbox_inches='tight', pad_inches=0.05, dpi=150)

    plt.show()'''


if __name__ == '__main__':
    main()
