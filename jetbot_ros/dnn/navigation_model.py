#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil
import numpy as np
import cv2
import math

import torch
import torch.nn as nn

import torchvision.transforms as transforms
import torchvision.datasets as datasets
import torchvision.models as models

from torch.optim.lr_scheduler import StepLR, ReduceLROnPlateau
from datetime import datetime

from jetbot_ros.dnn.reshape_model import reshape_model
from jetbot_ros.dnn.xy_dataset import XYDataset


model_names = sorted(name for name in models.__dict__
    if name.islower() and not name.startswith("__")
    and callable(models.__dict__[name]))

torch.manual_seed(1)  # reproducibility


class NavigationModel:
    """
    Enhanced model for navigation with obstacle avoidance capabilities
    """
    def __init__(self, model, type='regression', resolution=224, warmup=5):
        """
        Create or load a model with enhanced navigation capabilities.
        """
        if type != 'classification' and type != 'regression':
            raise ValueError("type must be 'classification' or 'regression' (was '{type}')")
            
        self.type = type
        self.resolution = resolution
        
        # 原有的資料轉換
        self.data_transforms = transforms.Compose([
                transforms.Resize((self.resolution, self.resolution)),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                     std=[0.229, 0.224, 0.225]),
            ])
        
        # 新增：避障相關參數
        self.obstacle_detection_params = {
            'edge_threshold_low': 15,
            'edge_threshold_high': 45,
            'texture_sensitivity': 25.0,
            'brightness_baseline': 60,
            'brightness_sensitivity': 40.0,
            'gradient_sensitivity': 15.0
        }
        
        # 新增：避障權重
        self.avoidance_weights = {
            'edge_score': 0.35,
            'texture_score': 0.25,
            'brightness_score': 0.20,
            'gradient_score': 0.20
        }
            
        # 載入模型（保持原有邏輯）
        if len(os.path.splitext(model)[1]) > 0:
            print(f"=> loading model '{model}'")
            checkpoint = torch.load(model)
            
            if self.type != checkpoint['type']:
                raise ValueError(f"'{model}' is a {checkpoint['type']} model, but expected a {self.type} model")
                
            print(f"     - arch           {checkpoint['arch']}")
            print(f"     - type           {checkpoint['type']}")
            print(f"     - outputs        {checkpoint['num_outputs']}")
            print(f"     - train loss     {checkpoint['train_loss']:.8f}")
            print(f"     - val loss       {checkpoint['val_loss']:.8f}")
            
            if self.classification:
                print(f"     - train accuracy {checkpoint['train_accuracy']:.8f}")
                print(f"     - val accuracy   {checkpoint['val_accuracy']:.8f}")

            self.model_arch = checkpoint['arch']
            self.num_outputs = checkpoint['num_outputs']
            
            self.model = models.__dict__[self.model_arch](pretrained=False)
            self.model = reshape_model(self.model, self.model_arch, self.num_outputs)
            
            self.model.load_state_dict(checkpoint['state_dict'])
            self.model.cuda()
        else:
            print(f"=> creating model '{model}'")
            self.model = models.__dict__[model](pretrained=True)
            self.model_arch = model
            self.num_outputs = 1000    # default classes for torchvision models
    
        self.model.cuda()
        
        # warmup model inference
        print(f"=> running model warm-up")
        
        for i in range(warmup):
            self.model.eval()
            
            with torch.no_grad():
                input = torch.ones((1, 3, resolution, resolution)).cuda()
                output = self.model(input)
        
        print(f"=> done with model warm-up")

    # === 原有的屬性和方法 ===
    @property
    def classification(self):
        return self.type == 'classification'
      
    @property
    def regression(self):
        return self.type == 'regression'
        
    def infer(self, image):
        """原有的 DNN 推理方法"""
        image = self.data_transforms(image).unsqueeze(0).cuda()

        self.model.eval()
        
        with torch.no_grad():
            output = self.model(image)
            
            if self.classification:
                output = nn.functional.softmax(output)
                prob, cls = torch.max(output, 1)
                return cls.item(), prob.item()
            else:
                return output.detach().squeeze().cpu().numpy() if output.requires_grad else output.squeeze().cpu().numpy()

    # === 新增的避障功能 ===
    
    def detect_obstacles_single_camera(self, image):
        """單攝影機障礙物檢測"""
        try:
            if image is None:
                return {'left': 1.0, 'fleft': 1.0, 'front': 1.0, 'fright': 1.0, 'right': 1.0}
            
            height, width, _ = image.shape
            
            # 定義檢測區域
            regions = {}
            regions['left'] = self._detect_obstacle_in_region(image, 0, width//3, height//3, 2*height//3)
            regions['fleft'] = self._detect_obstacle_in_region(image, width//4, width//2, height//3, 2*height//3)
            regions['front'] = self._detect_obstacle_in_region(image, width//3, 2*width//3, height//3, 2*height//3)
            regions['fright'] = self._detect_obstacle_in_region(image, width//2, 3*width//4, height//3, 2*height//3)
            regions['right'] = self._detect_obstacle_in_region(image, 2*width//3, width, height//3, 2*height//3)
            
            return regions
            
        except Exception as e:
            print(f"Single camera obstacle detection error: {str(e)}")
            return {'left': 1.0, 'fleft': 1.0, 'front': 1.0, 'fright': 1.0, 'right': 1.0}

    def detect_obstacles_dual_camera(self, left_image, right_image):
        """雙攝影機障礙物檢測"""
        try:
            if left_image is None or right_image is None:
                return {'left': 1.0, 'fleft': 1.0, 'front': 1.0, 'fright': 1.0, 'right': 1.0}
            
            # 左攝影機檢測
            left_regions = self.detect_obstacles_single_camera(left_image)
            
            # 右攝影機檢測
            right_regions = self.detect_obstacles_single_camera(right_image)
            
            # 融合雙攝影機結果
            regions = {}
            regions['left'] = left_regions['left']  # 純左側用左攝影機
            regions['fleft'] = min(left_regions['fright'], left_regions['front'])  # 左前方
            regions['front'] = min(left_regions['front'], right_regions['front'])  # 前方取最小值
            regions['fright'] = min(right_regions['fleft'], right_regions['front'])  # 右前方
            regions['right'] = right_regions['right']  # 純右側用右攝影機
            
            return regions
            
        except Exception as e:
            print(f"Dual camera obstacle detection error: {str(e)}")
            return {'left': 1.0, 'fleft': 1.0, 'front': 1.0, 'fright': 1.0, 'right': 1.0}

    def _detect_obstacle_in_region(self, image, x1, x2, y1, y2):
        """在指定區域檢測障礙物"""
        try:
            # 提取 ROI
            roi = image[y1:y2, x1:x2]
            if roi.size == 0:
                return 1.0
            
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # 1. 邊緣檢測
            blurred = cv2.GaussianBlur(gray, (3, 3), 0)
            edges = cv2.Canny(blurred, 
                             self.obstacle_detection_params['edge_threshold_low'],
                             self.obstacle_detection_params['edge_threshold_high'])
            edge_count = cv2.countNonZero(edges)
            edge_ratio = edge_count / edges.size if edges.size > 0 else 0
            edge_score = edge_ratio * 100
            
            # 2. 紋理檢測
            std_dev = np.std(gray.astype(np.float32))
            texture_score = min(std_dev / self.obstacle_detection_params['texture_sensitivity'], 1.0) * 100
            
            # 3. 亮度檢測
            mean_brightness = np.mean(gray)
            brightness_diff = abs(mean_brightness - self.obstacle_detection_params['brightness_baseline'])
            brightness_score = min(brightness_diff / self.obstacle_detection_params['brightness_sensitivity'], 1.0) * 100
            
            # 4. 梯度檢測
            sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
            sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
            gradient_magnitude = np.sqrt(sobelx**2 + sobely**2)
            gradient_score = min(np.mean(gradient_magnitude) / self.obstacle_detection_params['gradient_sensitivity'], 1.0) * 100
            
            # 5. 綜合評分
            obstacle_score = (
                edge_score * self.avoidance_weights['edge_score'] + 
                texture_score * self.avoidance_weights['texture_score'] + 
                brightness_score * self.avoidance_weights['brightness_score'] + 
                gradient_score * self.avoidance_weights['gradient_score']
            )
            
            # 轉換為距離值（1.0 = 無障礙物，0.0 = 有障礙物）
            simulated_distance = max(0.0, 1.0 - obstacle_score / 100.0)
            
            return simulated_distance
            
        except Exception as e:
            print(f"Region obstacle detection error: {str(e)}")
            return 1.0

    def calculate_avoidance_vector(self, regions, avoidance_threshold=0.5):
        """根據障礙物檢測結果計算避障向量"""
        try:
            # 檢測各區域是否有障礙物
            front_close = regions['front'] < avoidance_threshold
            fleft_close = regions['fleft'] < avoidance_threshold
            fright_close = regions['fright'] < avoidance_threshold
            left_close = regions['left'] < avoidance_threshold
            right_close = regions['right'] < avoidance_threshold
            
            # 初始化避障指令
            avoid_linear = 0.0
            avoid_angular = 0.0
            
            # 避障邏輯
            if front_close and not fleft_close and not fright_close:
                # 正前方有障礙物，左右都沒有 -> 右轉
                avoid_linear = 0.1
                avoid_angular = -0.3
            elif not front_close and fleft_close and not fright_close:
                # 左前方有障礙物 -> 右轉
                avoid_linear = 0.15
                avoid_angular = -0.2
            elif not front_close and not fleft_close and fright_close:
                # 右前方有障礙物 -> 左轉
                avoid_linear = 0.15
                avoid_angular = 0.2
            elif front_close and fleft_close and not fright_close:
                # 正前方和左前方都有障礙物 -> 大幅右轉
                avoid_linear = 0.05
                avoid_angular = -0.4
            elif front_close and not fleft_close and fright_close:
                # 正前方和右前方都有障礙物 -> 大幅左轉
                avoid_linear = 0.05
                avoid_angular = 0.4
            elif not front_close and fleft_close and fright_close:
                # 左右前方都有障礙物，正前方沒有 -> 直行
                avoid_linear = 0.2
                avoid_angular = 0.0
            elif front_close and fleft_close and fright_close:
                # 三個方向都有障礙物 -> 後退並轉向
                avoid_linear = -0.1
                avoid_angular = -0.3
            else:
                # 沒有障礙物或只有左右側障礙物 -> 正常前進
                avoid_linear = 0.0
                avoid_angular = 0.0
            
            return avoid_linear, avoid_angular
            
        except Exception as e:
            print(f"Avoidance vector calculation error: {str(e)}")
            return 0.0, 0.0

    def update_obstacle_detection_params(self, **kwargs):
        """更新障礙物檢測參數"""
        for key, value in kwargs.items():
            if key in self.obstacle_detection_params:
                self.obstacle_detection_params[key] = value
                print(f"Updated {key} to {value}")
            else:
                print(f"Unknown parameter: {key}")

    def update_avoidance_weights(self, **kwargs):
        """更新避障權重"""
        for key, value in kwargs.items():
            if key in self.avoidance_weights:
                self.avoidance_weights[key] = value
                print(f"Updated weight {key} to {value}")
            else:
                print(f"Unknown weight: {key}")

    def get_obstacle_detection_info(self):
        """獲取障礙物檢測參數信息"""
        return {
            'params': self.obstacle_detection_params.copy(),
            'weights': self.avoidance_weights.copy()
        }

    # === 保持原有的訓練和其他方法 ===
    
    def train(self, dataset, epochs=10, batch_size=1, learning_rate=0.01, scheduler='StepLR_75', 
              workers=1, train_split=0.8, print_freq=10, use_class_weights=True, 
              save=f"data/models/{datetime.now().strftime('%Y%m%d%H%M')}"):
        """
        Train the model on a dataset (原有方法保持不變)
        """
        train_loader, val_loader, class_weights = self.load_dataset(dataset, batch_size, workers, train_split)
        
        self.model.cuda()
        
        # setup model, loss function, and solver
        if self.classification:
            criterion = nn.CrossEntropyLoss(weight=torch.Tensor(class_weights) if use_class_weights else None).cuda()
        else:
            criterion = nn.MSELoss()
            
        optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)
        scheduler = self._create_scheduler(scheduler, optimizer)        
        best_metric = -np.inf if self.classification else np.inf
        
        # train for the specified number of epochs
        for epoch in range(epochs):
            self.model.train()
            
            train_loss = 0.0
            train_accuracy = 0.0
            
            for i, (images, target) in enumerate(train_loader):
                images = images.cuda(non_blocking=True)
                target = target.cuda(non_blocking=True)
                
                # compute model output
                output = self.model(images)
                loss = criterion(output, target)
                
                # compute gradient and do solver step
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
        
                # keep track of accuracy/loss over the batch
                accuracy = self.accuracy(output, target) if self.classification else 0.0
                train_accuracy += accuracy
                train_loss += loss
                
                if i % print_freq == 0:
                    if self.classification:
                        print(f"Epoch {epoch}:  train=[{i}/{len(train_loader)}]  lr={scheduler._last_lr[0]:.2g}  loss={train_loss/(i+1):.8f}  accuracy={train_accuracy/(i+1):.8f}")
                    else:
                        print(f"Epoch {epoch}:  train=[{i}/{len(train_loader)}]  lr={scheduler._last_lr[0]:.2g}  loss={train_loss/(i+1):.8f}")
  
            if isinstance(scheduler, ReduceLROnPlateau):
                scheduler.step(metrics=train_loss)
            else:
                scheduler.step()
                
            train_loss /= len(train_loader)
            train_accuracy /= len(train_loader)
                    
            if val_loader is not None:
                val_loss, val_accuracy = self.validate(val_loader, criterion, epoch, print_freq)
            else:
                val_loss = train_loss
                val_accuracy = train_accuracy
                
            if self.classification:
                print(f"Epoch {epoch}:  train_loss={train_loss:.8f}  train_accuracy={train_accuracy:.8f}")
                print(f"Epoch {epoch}:  val_loss={val_loss:.8f}  val_accuracy={val_accuracy:.8f}")
            else:
                print(f"Epoch {epoch}:  train_loss={train_loss:.8f}")
                print(f"Epoch {epoch}:  val_loss={val_loss:.8f}")
                
            if save:
                checkpoint = {
                    'epoch': epoch,
                    'arch': self.model_arch,
                    'type': self.type,
                    'resolution': self.resolution,
                    'num_outputs': self.num_outputs,
                    'state_dict': self.model.state_dict(),
                    'train_loss': train_loss.item(),
                    'val_loss': val_loss.item(),
                }
                
                if self.classification:
                    checkpoint['train_accuracy'] = train_accuracy
                    checkpoint['val_accuracy'] = val_accuracy
                    
                is_best = val_accuracy > best_metric if self.classification else val_loss < best_metric
                self.save_checkpoint(checkpoint, is_best, save)
            
            if self.classification:
                best_metric = max(val_accuracy, best_metric)
            else:
                best_metric = min(val_loss, best_metric)

    def validate(self, val_loader, criterion, epoch, print_freq=10):
        """
        Measure model performance on the val dataset (原有方法)
        """
        self.model.eval()
        
        val_loss = 0.0
        val_accuracy = 0.0
        
        with torch.no_grad():
            for i, (images, target) in enumerate(val_loader):
                images = images.cuda(non_blocking=True)
                target = target.cuda(non_blocking=True)
                
                # compute model output
                output = self.model(images)
                loss = criterion(output, target)
                
                # update accuracy and loss
                accuracy = self.accuracy(output, target) if self.classification else 0.0
                val_accuracy += accuracy
                val_loss += loss
                
                if i % print_freq == 0:
                    if self.classification:
                        print(f"Epoch {epoch}:  val=[{i}/{len(val_loader)}]  loss={val_loss/(i+1):.8f}  accuracy={val_accuracy/(i+1):.8f}")
                    else:
                        print(f"Epoch {epoch}:  val=[{i}/{len(val_loader)}]  loss={val_loss/(i+1):.8f}")
                    
        val_loss /= len(val_loader)
        val_accuracy /= len(val_loader)
        
        return val_loss, val_accuracy
        
    def accuracy(self, output, target):
        """
        Compute the classification accuracy.
        """
        _, preds = torch.max(output, 1)
        return (preds == target).float().mean().cpu().item()            
    
    def save_checkpoint(self, state, is_best, path=None, filename='checkpoint.pth', best_filename='model_best.pth'):
        """
        Save a model checkpoint file, along with the best-performing model if applicable
        """
        if path:
            filename = os.path.join(path, filename)
            best_filename = os.path.join(path, best_filename)
            
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        os.makedirs(os.path.dirname(best_filename), exist_ok=True)
 
        # save the checkpoint
        torch.save(state, filename)

        # earmark the best checkpoint
        if is_best:
            shutil.copyfile(filename, best_filename)
            print("saved best model to:  " + best_filename)
        else:
            print("saved checkpoint to:  " + filename)
        
    def load_dataset(self, dataset, batch_size=2, workers=1, train_split=0.8):
        """
        Load dataset from the specified path (原有方法)
        """
        normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                         std=[0.229, 0.224, 0.225])
                            
        if self.type == 'classification':
            dataset = datasets.ImageFolder(dataset, self.data_transforms)
        elif self.type == 'regression':
            dataset = XYDataset(dataset, self.data_transforms)
            
        # split into train/val
        if train_split > 0:
            lengths = [int(len(dataset) * train_split)]
            lengths.append(len(dataset) - lengths[0])

            train_dataset, val_dataset = torch.utils.data.random_split(
                dataset, lengths, 
                generator=torch.Generator().manual_seed(1))
                
            val_loader = torch.utils.data.DataLoader(
                val_dataset, batch_size=batch_size, num_workers=workers,
                shuffle=False, pin_memory=True)
        else:
            train_dataset = dataset
            val_dataset = None
            val_loader = None
            
        # create dataloaders
        train_loader = torch.utils.data.DataLoader(
            train_dataset, batch_size=batch_size, num_workers=workers,
            shuffle=True, pin_memory=True)

        print(f'=> train samples:   {len(train_dataset)}')
        print(f'=> val samples:     {len(val_dataset) if val_dataset is not None else 0}')
        
        # reshape model if needed   
        if self.type == 'classification':
            num_outputs = len(dataset.classes)
            print(f'=> dataset classes: {len(dataset.classes)} ({str(dataset.classes)})')
        else:
            num_outputs = 2
            
        if self.num_outputs != num_outputs:
            self.model = reshape_model(self.model, self.model_arch, num_outputs)
            self.num_outputs = num_outputs

        # get class weights
        if self.type == 'classification':
            class_weights, class_counts = self.get_class_weights(dataset)
            
            print('=> class distribution:')
            
            for idx, (weight, count) in enumerate(zip(class_weights, class_counts)):
                print(f'     [{idx}] - {count} samples ({count/sum(class_counts):.4f}), weight {weight:.8f}')
        else:
            class_weights = [1.0] * self.num_outputs
            
        return train_loader, val_loader, class_weights
 
    def get_class_weights(self, dataset):
        counts = [0] * len(dataset.classes) 
        weights = [0.] * len(dataset.classes)   
        
        for item in dataset.imgs:                                                         
            counts[item[1]] += 1                                                     
                                            
        max_count = max(counts)
        
        for i in range(len(dataset.classes)):   
            if counts[i] > 0:
                weights[i] = max_count / counts[i]   
            else:
                weights[i] = 1.0
                
        return weights, counts          

    @staticmethod
    def _create_scheduler(scheduler, optimizer):
        """
        Create a scheduler from a param string like 'StepLR_30'
        """
        if scheduler.startswith('StepLR'):
            return StepLR(optimizer, step_size=NavigationModel._parse_param(scheduler, default=30))
        elif scheduler.startswith('ReduceLROnPlateau'):
            return ReduceLROnPlateau(optimizer, patience=NavigationModel._parse_param(scheduler, default=10))
        else:
            raise ValueError(f"invalid scheduler '{scheduler}'") 
        
    @staticmethod
    def _parse_param(str, default):
        """
        Parse a parameter in a string of the form 'text_value'
        """
        idx = str.find('_')
        
        if idx < 0 or idx == (len(str) - 1):
            return default

        return int(str[idx+1:])