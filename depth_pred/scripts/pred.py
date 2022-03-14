#!/usr/bin/env python3

import argparse
import torch
import torch.nn as nn
import torch.nn.parallel
import torch.utils.data
import AnyNet.utils.logger as logger
from AnyNet.models.anynet import AnyNet
import os
import PIL
import numpy as np
from AnyNet.dataloader import preprocess
import rospy
from sensor_msgs.msg import Image
import message_filters as mf
from cv_bridge import CvBridge
import cv2 as cv
import queue

parser = argparse.ArgumentParser(description='Anynet fintune on KITTI')
parser.add_argument('--maxdisp', type=int, default=192,
                    help='maxium disparity')
parser.add_argument('--loss_weights', type=float, nargs='+', default=[0.25, 0.5, 1., 1.])
parser.add_argument('--max_disparity', type=int, default=192)
parser.add_argument('--maxdisplist', type=int, nargs='+', default=[12, 3, 3])
parser.add_argument('--datatype', default='2015',
                    help='datapath')
parser.add_argument('--datapath', default=None, help='datapath')
parser.add_argument('--save_path', type=str, default='results/finetune_anynet',
                    help='the path of saving checkpoints and log')
parser.add_argument('--with_spn', action='store_true', help='with spn network or not', default=False)
parser.add_argument('--print_freq', type=int, default=5, help='print frequence')
parser.add_argument('--init_channels', type=int, default=1, help='initial channels for 2d feature extractor')
parser.add_argument('--nblocks', type=int, default=2, help='number of layers in each stage')
parser.add_argument('--channels_3d', type=int, default=4, help='number of initial channels 3d feature extractor ')
parser.add_argument('--layers_3d', type=int, default=4, help='number of initial layers in 3d network')
parser.add_argument('--growth_rate', type=int, nargs='+', default=[4, 1, 1], help='growth rate in the 3d network')
parser.add_argument('--spn_init_channels', type=int, default=8, help='initial channels for spnet')
parser.add_argument('--start_epoch_for_spn', type=int, default=121)
parser.add_argument('--pretrained', type=str, default='../model/checkpoint.tar',
                    help='pretrained model path')
parser.add_argument('--evaluate', action='store_true')




left_img_queue = queue.Queue(maxsize=1)
right_img_queue = queue.Queue(maxsize=1)


def left_img_cb(img:Image):
    global left_img_queue
    left_img_queue.put(img)
    return


def right_img_cb(img:Image):
    global right_img_queue
    right_img_queue.put(img)
    return




# def stereo_cb(left: Image, right: Image):
#     global model, transform, bridge
#     left_img = PIL.Image.fromarray(bridge.imgmsg_to_cv2(left)).convert("RGB")
#     right_img = PIL.Image.fromarray(bridge.imgmsg_to_cv2(right)).convert("RGB")
#     left_img = transform(left_img).float().cuda()
#     right_img = transform(right_img).float().cuda()
#     left_img = torch.unsqueeze(left_img, 0)
#     right_img = torch.unsqueeze(right_img, 0)
#     with torch.no_grad():
#         outputs = model(left_img, right_img)
#         depth_img = bridge.cv2_to_imgmsg((outputs[0][0, :, :].squeeze().cpu().numpy()))
#
#     depth_img.header.stamp = left.header.stamp
#     depth_pub.publish(depth_img)




# from dataloader import diy_dataset as ls

# def main():
#     global left_img_queue, right_img_queue
bridge = CvBridge()
args = parser.parse_args()

rospy.init_node("depth_pred_node", anonymous=True)

left_img_sub = rospy.Subscriber("/depth_img/left", Image, left_img_cb)
right_img_sub = rospy.Subscriber("/depth_img/right", Image, right_img_cb)

depth_pub = rospy.Publisher("/airsim_node/car/left/depth", Image, queue_size=1)

log = logger.setup_logger(args.save_path + '/training.log')
transform = preprocess.get_transform(augment=False)

model = AnyNet(args)
model = nn.DataParallel(model).cuda()

if args.pretrained:
    if os.path.isfile(args.pretrained):
        checkpoint = torch.load(args.pretrained)
        model.load_state_dict(checkpoint['state_dict'], strict=False)
        print("=> loaded pretrained model '{}'"
              .format(args.pretrained))
    else:
        raise FileNotFoundError
else:
    raise ValueError

model.eval()
while not rospy.is_shutdown():
    # print(right_img_queue.qsize())
    while not right_img_queue.empty() and not left_img_queue.empty() and \
            left_img_queue.queue[0].header.stamp.to_sec() - right_img_queue.queue[0].header.stamp.to_sec() < -0.02:
        left_img_queue.get()
        # print("throw l")
    while not right_img_queue.empty() and not left_img_queue.empty() and \
            left_img_queue.queue[0].header.stamp.to_sec() - right_img_queue.queue[0].header.stamp.to_sec() > 0.02:
        right_img_queue.get()
    if left_img_queue.empty() or right_img_queue.empty():
        # print("continue")
        continue
    # print("processing")
    left_img = left_img_queue.get()
    stamp = left_img.header.stamp
    left_img = PIL.Image.fromarray(bridge.imgmsg_to_cv2(left_img)).convert("RGB")

    right_img = PIL.Image.fromarray(bridge.imgmsg_to_cv2(right_img_queue.get())).convert("RGB")
    left_img = transform(left_img).float().cuda()
    right_img = transform(right_img).float().cuda()
    left_img = torch.unsqueeze(left_img, 0)
    right_img = torch.unsqueeze(right_img, 0)
    with torch.no_grad():
        outputs = model(left_img, right_img)
        depth_img = bridge.cv2_to_imgmsg((outputs[-1][0, :, :].squeeze().cpu().numpy()))

    depth_img.header.stamp = stamp
    depth_pub.publish(depth_img)
    # ts = mf.TimeSynchronizer([left_img_sub, right_img_sub], 1)
    #
    # ts.registerCallback(stereo_cb)
    # rospy.spin()


# def test(dataloader, model, log):
#     model.eval()
#     j = 0
#
#     left_img = PIL.Image.open('/home/lzh/projects/AnyNet/dataset/training/image_2/72.png').convert('RGB')
#     right_img = PIL.Image.open('/home/lzh/projects/AnyNet/dataset/training/image_3/72.png').convert('RGB')
#
#     processed = preprocess.get_transform(augment=False)
#     left_img = processed(left_img).float().cuda()
#     left_img = torch.unsqueeze(left_img, 0)
#     right_img = processed(right_img).float().cuda()
#     right_img = torch.unsqueeze(right_img, 0)
#     with torch.no_grad():
#         outputs = model(left_img, right_img)
#         PIL.Image.fromarray((outputs[-1][0, 0, :, :].squeeze().cpu().numpy() * 256).astype(np.uint16)).save(
#             args.save_path + '/{}.png'.format(j))

if __name__ == '__main__':
    main()
