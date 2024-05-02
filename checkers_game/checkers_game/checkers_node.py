import re
import cv2
import pygame
from checkers_game.constants import WIDTH, HEIGHT, SQUARE_SIZE, RED, WHITE, BLACK
from camera.ximea_camera import XimeaCamera
from checkers_game.camera.board_detection import BoardDetection
from checkers_game.checkers.game import Game
from checkers_game.minimax.algorithm import WHITE, minimax
import mediapipe as mp
import time
import numpy as np

from checkers_msgs.msg import Board, Piece, Move, HandDetected, RobotMove

import rclpy
from rclpy.node import Node


class CheckersNode(Node):
    def __init__(self):
        super().__init__('checkers_game')
        pygame.init()
        self.WIN = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Checkers")
        self.clock = pygame.time.Clock()
        self.FPS = 60
        self.ximeaCamera = XimeaCamera()
        self.boardDetection = BoardDetection(self.ximeaCamera)
        self.game = Game(self.WIN, self.boardDetection)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.mp_drawing = mp.solutions.drawing_utils
        self.wait = False
        self.isRobotMoveDone = True

        self.board_publisher = self.create_publisher(Board, 'board_topic', 10)
        self.move_publisher = self.create_publisher(Move, 'move_topic', 10)
        self.hand_detected_publisher = self.create_publisher(HandDetected, 'hand_detected', 10)
        self.robot_move_subscription = self.create_subscription(RobotMove, '/robot_move_topic', self.robot_move_callback, 10)

        self.oldCameraImage = self.ximeaCamera.get_camera_image()
        self.waiting_for_true = True
        self.last_false_time = None
        self.isPlayerMoveDone = False

        # Create a ROS2 timer to handle pygame events every 1/FPS seconds
        self.timer = self.create_timer(1/self.FPS, self.update_pygame)


    def robot_move_callback(self, msg):
        print("Robot dokoncil svoj tah")
        self.isRobotMoveDone = msg.robot_move_done

    def update_pygame(self):

        # This is your game's main loop controlled by ROS2 timer
        if pygame.event.get(pygame.QUIT):
            self.destroy_node()
            pygame.quit()
            return

        self.clock.tick(self.FPS)
        cameraImage = self.ximeaCamera.get_camera_image()
        board_coordinates = self.get_board_coordinates()
        isHandDetected = self.detect_hand(cameraImage, board_coordinates)
        boardDetected = self.boardDetection.get_board(cameraImage, self.game)

        if isHandDetected:
            # print("HAAANDDD")
            self.publish_hand_detected_state()

            
        isChange = self.findDifference(cameraImage, self.oldCameraImage)
        self.oldCameraImage = cameraImage

        if self.isRobotMoveDone:
            if self.waiting_for_true:
                if isChange:
                    self.waiting_for_true = False
                    # print("Detected first change, now waiting for false or doing nothing.")
            else:
                if isChange:
                    # Reset the last_false_time if true is detected again
                    self.last_false_time = None
                    # print("Change detected, continue waiting for a potential false.")
                else:
                    if self.last_false_time is None:
                        self.last_false_time = time.time()
                    elif time.time() - self.last_false_time > 1:
                        # If it's been more than 5 seconds since the last false
                        # and no true has reset this, do the "some stuff"
                        # print("Performing some stuff as no true detected within 5 seconds after false.")
                        self.isPlayerMoveDone = True
                        self.last_false_time = None  # Reset the last_false_time
                        self.waiting_for_true = True  # Reset to start waiting for true again

            # if self.game.winner() is not None:
            #         print(self.game.winner())
            #         break
                
            # if(self.game.wasItValidMove(boardDetected)):
            #     self.game.update(boardDetected)

            #     self.publish_board_state()
                
            #     if self.game.board.isBoardCreated == True and self.game.turn == BLACK and self.game.aiMove == None:
            #             print("AI ZACINA")
            #             value, new_board, new_move, new_piece, removed = minimax(self.game.get_board(), 3, BLACK, self)
            #             self.aiMove = new_move
            #             self.aiPiece = new_piece
            #             self.aiPiece.color = BLACK
            #             self.game.update(new_board.board)
            #             self.publish_move_state(new_move, new_piece, removed)
            #             self.isRobotMoveDone = False;
            # else:
            #     print("Neplatny TAH")
                
            if self.isPlayerMoveDone or not self.wait:
                if self.game.winner() is not None:
                    print(self.game.winner())
                
                
                if(self.game.wasItValidMove(boardDetected)):
                    self.game.update(boardDetected)

                    self.publish_board_state()
                    
                    if self.game.board.isBoardCreated == True and self.game.turn == BLACK and self.game.aiMove == None:
                            print("AI ZACINA")
                            value, new_board, new_move, new_piece, removed = minimax(self.game.get_board(), 3, BLACK, self)
                            self.aiMove = new_move
                            self.aiPiece = new_piece
                            self.aiPiece.color = BLACK
                            self.game.update(new_board.board)
                            self.publish_move_state(new_move, new_piece, removed)
                            self.isRobotMoveDone = False;
                else:
                    print("Neplatny TAH")

                self.isPlayerMoveDone = False
                self.wait = True 
    
            key = cv2.waitKey(1) & 0xFF
            if key == 32:
                self.wait = False

    def findDifference(self, newCameraImage, oldCameraImage):
        image2 = oldCameraImage
        image1 = newCameraImage

        diff = cv2.absdiff(image1, image2)

        # Threshold the diff image so we get only significant changes
        _, thresh = cv2.threshold(diff, 10, 255, cv2.THRESH_BINARY)

        # Optionally apply some dilation to make the differences more visible
        kernel = np.ones((5,5), np.uint8)
        thresh = cv2.dilate(thresh, kernel, iterations=1)

        # Convert to grayscale if the images are color images
        if len(thresh.shape) == 3:
            thresh = cv2.cvtColor(thresh, cv2.COLOR_BGR2GRAY)

        # Calculate the percentage of change
        change_percentage = np.sum(thresh == 255) / thresh.size * 100

        # Define the threshold for significant change (e.g., 1% of the image)
        significant_change_threshold = 1  # 1 percent of the image area

        # Determine the message based on the amount of change
        # print("Image Similarity: {:.4f}%".format(change_percentage))

        isChange = False
        if change_percentage > significant_change_threshold:
            message = "change detected"
            isChange = True
        else:
            message = "No change"
            isChange = False

        # Display the result with text
        cv2.putText(thresh, message, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow('Difference', thresh)

        return isChange

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True

    def get_board_coordinates(self):
        board_coordinates = []
        for boundary in self.boardDetection.bounderies:
            x, y, w, h = cv2.boundingRect(boundary)
            board_coordinates = [x, y, w, h]
        return board_coordinates

    def detect_hand(self, cameraImage, board_coordinates):
        bgr_frame = cv2.cvtColor(cameraImage, cv2.COLOR_RGB2BGR)
        rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        if results.multi_hand_landmarks:
            for landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(rgb_frame, landmarks, self.mp_hands.HAND_CONNECTIONS)
                min_x, min_y, max_x, max_y = self.get_hand_bounding_box(landmarks, rgb_frame)
                cv2.rectangle(rgb_frame, (min_x, min_y), (max_x, max_y), (0, 255, 0), 2)
                cv2.imshow("Hand Tracking", rgb_frame)
                if self.do_rectangles_overlap((min_x, min_y, max_x - min_x, max_y - min_y), board_coordinates):
                    print("Hand overlaps with the rectangle!")
                    return True
                else:
                    print("Hand does not overlap with the rectangle!")
                    return False
        cv2.imshow("Hand Tracking", rgb_frame)
        return False

    @staticmethod
    def do_rectangles_overlap(rect1, rect2, threshold=50):
        if len(rect1) != 4 or len(rect2) != 4:
            return False
        
        x1, y1, w1, h1 = rect1
        x2, y2, w2, h2 = rect2
        return (x1 - threshold < x2 + w2) and (x1 + w1 + threshold > x2) and (y1 - threshold < y2 + h2) and (y1 + h1 + threshold > y2)

    @staticmethod
    def get_hand_bounding_box(landmarks, frame):
        min_x = int(min(landmark.x for landmark in landmarks.landmark) * frame.shape[1])
        max_x = int(max(landmark.x for landmark in landmarks.landmark) * frame.shape[1])
        min_y = int(min(landmark.y for landmark in landmarks.landmark) * frame.shape[0])
        max_y = int(max(landmark.y for landmark in landmarks.landmark) * frame.shape[0])
        return min_x, min_y, max_x, max_y


    def color_to_string(self, color):
        if color == WHITE:
            return "white"
        elif color == BLACK:
            return "red"
        
    def publish_board_state(self):    
        board_msg = Board()
        for piece in self.game.board.get_all_pieces():
            piece_msg = Piece()
            piece_msg.row = piece.row
            piece_msg.col = piece.col
            piece_msg.color = self.color_to_string(piece.color)
            piece_msg.king = piece.king
            # Set x and y if needed, depending on how your Piece class defines them
            board_msg.pieces.append(piece_msg)
        self.board_publisher.publish(board_msg)

    def publish_move_state(self, new_move, new_piece, removed):
        move_msg = Move()
        move_msg.target_row = new_move[0]
        move_msg.target_col = new_move[1]

        piece_msg = Piece()
        piece_msg.row = new_piece.row
        piece_msg.col = new_piece.col
        piece_msg.color = self.color_to_string(new_piece.color)
        piece_msg.king = new_piece.king
        move_msg.piece_for_moving = piece_msg
        
        for piece in removed:
            piece_msg = Piece()
            piece_msg.row = piece.row
            piece_msg.col = piece.col
            piece_msg.color = self.color_to_string(piece.color)
            piece_msg.king = piece.king
            # Set x and y if needed, depending on how your Piece class defines them
            move_msg.removed_pieces.append(piece_msg)

        self.move_publisher.publish(move_msg)

    def publish_hand_detected_state(self):
        hand_detected_msg = HandDetected()
        hand_detected_msg.hand_detected = True
        # self.hand_detected_publisher.publish(hand_detected_msg)


def main(args=None):
    rclpy.init(args=args)
    checkers_node = CheckersNode()
    rclpy.spin(checkers_node)  # This will now also periodically call update_pygame()

    checkers_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()