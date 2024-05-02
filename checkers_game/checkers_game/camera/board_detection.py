
import cv2
from matplotlib.pyplot import imshow
from camera.ximea_camera import XimeaCamera
import numpy as np
import copy
from checkers.piece import Piece
from constants import BLACK, ROWS, RED, SQUARE_SIZE, COLS, WHITE, GREY, BROWN

class BoardDetection:

    def __init__(self, ximeaCamera):
        self.ximeaCamera = ximeaCamera
        self._init()
        

    def _init(self):
        self.numberOfEmptyFields = 40
        self.param1ForGetAllContours = 255
        self.bounderies = self._get_trim_param()

        cameraImage = self.ximeaCamera.get_camera_image()
        cameraImage = self._trim_image(cameraImage, self.bounderies)

        self.gameBoardFieldsContours = self._get_contours(cameraImage)
        

    def _get_trim_param(self):
        bounderies = []
        temp_bounderies = []
        while 1:
            temp_bounderies = []
            cameraImage = self.ximeaCamera.get_camera_image()
            cameraImage = self._trim_image(cameraImage, bounderies)
            cv2.imshow("original", cameraImage)

            temp_bounderies.append(self._get_bounderies(cameraImage))
            
            # Get the bounding rectangle for the largest contour
            x, y, w, h = cv2.boundingRect(temp_bounderies[-1])

            # Crop the image using the bounding rectangle
            trimmed_image = cameraImage.copy()
            trimmed_image = trimmed_image[y:y+h, x:x+w]

            cv2.imshow("trimming", trimmed_image)

            key = cv2.waitKey(1) & 0xFF
            if key == 32:
                bounderies.extend(temp_bounderies)
            
            if key == 27:
                cv2.destroyWindow("trimming")
                cv2.destroyWindow("original")
                cv2.destroyWindow("contours")
                cv2.destroyWindow("thresh")

                break

        return bounderies

    def _get_bounderies(self, image):  # Remove argument here
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)

        # Threshold the image to separate the black frame
        thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

        # imagem = cv2.bitwise_not(thresh)

        cv2.imshow("thresh", thresh)

        # Find contours in the thresholded image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sort the contours by area and keep the largest one
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        result = image.copy()
        for c in contours:
            cv2.drawContours(result, [c], -1, (0, 255, 0), 2)
        cv2.imshow("contours", result)

        return contours[0]

    def _trim_image(self, image, bounderies):
        trimmed_image = image.copy()
        for boundery in bounderies:
            # Get the bounding rectangle for the largest contour
            x, y, w, h = cv2.boundingRect(boundery)

            # Crop the image using the bounding rectangle
            trimmed_image = trimmed_image[y:y+h, x:x+w]
            
        return trimmed_image
    

    def _get_contours(self, cameraImage):
        all_contours = self._get_contours_off_all_rectangles(cameraImage)
        return self._get_sorted_contours(all_contours)


    def _get_contours_off_all_rectangles(self, image):
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        
        # Apply edge detection
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        attempts = 1
        while 1:
            if(self.param1ForGetAllContours < 1):
                attempts += 1
                if(attempts > 1):
                    break
                
            self.param1ForGetAllContours -= 1

            # Detect lines using the Hough Line Transform
            lines = cv2.HoughLines(edges, 1, np.pi / 180, self.param1ForGetAllContours)

            # Create a copy of the original image to draw lines on
            black_image = image.copy()
            black_image = np.zeros_like(black_image)

            # Draw the lines on the image
            if lines is not None:
                for rho, theta in lines[:, 0]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * a)
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * a)

                    cv2.line(black_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

            gray = cv2.cvtColor(black_image, cv2.COLOR_RGB2GRAY)

            # cv2.imshow("all_rectangles_black_image", gray)

            # Find contours
            contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Filter for rectangles
            rectangles = []
            for cnt in contours:
                # Get convex hull
                hull = cv2.convexHull(cnt)
                
                # Get approximate polygon
                epsilon = 0.02 * cv2.arcLength(hull, True)
                approx = cv2.approxPolyDP(hull, epsilon, True)
                
                # Check if it is a rectangle
                if len(approx) == 4:
                    rectangles.append(approx)


            # Sort the contours by area and keep the largest one
            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            new_image = image.copy()
            if(len(contours) >= 64 and len(contours) <= 150):
                correct_result = 0
                new_contours = []
                for position in range(1, len(contours)):

                    x,y,w,h = cv2.boundingRect(contours[position])
                
                    if(w > 20 and h > 20 and correct_result < 64):
                        new_contours.append(contours[position])
                        correct_result += 1
                        cv2.rectangle(new_image, (x, y), (x + w, y + h), (36,255,12), 1)
                # cv2.drawContours(result, [contours[position]], -1, (0, 0, 255), 2)

                # cv2.imshow('all_rectangles_new_image', new_image)
                if(correct_result == 64):
                    break

        return new_contours

    def _get_sorted_contours(self, all_contours):
        array_2d_in_2d = []
        position = 0
        group_range = 25  # Rozsah pre skupiny
        if(len(all_contours) == 64):
            for i in range(8):
                inner_array = []
                for j in range(8):
                    x, y, w, h = cv2.boundingRect(all_contours[position])
                    inner_2d_array = [x, y, w, h]
                    inner_array.append(inner_2d_array)
                    position += 1
                array_2d_in_2d.append(inner_array)

        # Function to flatten and sort the 2D array based on "y" values
        def flatten_and_sort(array):
            flat_list = [item for sublist in array for item in sublist]
            return sorted(flat_list, key=lambda x: x[1])

        # Function to group elements by "y" value within a specified range
        def group_by_y_range(sorted_list, group_range):
            grouped = []
            while sorted_list:
                base_y = sorted_list[0][1]
                group = []

                for element in sorted_list:
                    if base_y - group_range <= element[1] <= base_y + group_range:
                        group.append(element)
                        if len(group) == 8:
                            break

                grouped.append(group)
                sorted_list = [x for x in sorted_list if x not in group]

            return grouped

        # Function to ensure the final array is 8x8
        def ensure_8x8_array(grouped_elements):
            while len(grouped_elements) < 8:
                grouped_elements.append([(0, 0)] * 8)
            return grouped_elements[:8]

        def order_rows_by_x(array_2d):
            ordered_array_2d = []
            for row in array_2d:
                ordered_row = sorted(row, key=lambda x: x[0])
                ordered_array_2d.append(ordered_row)
            return ordered_array_2d
        
        # Function to reorder the rows based on the first element's "y" value in each row
        def reorder_rows_by_first_y(array_2d):
            # Sorting the entire 2D array based on the "y" value of the first element in each row
            reordered_array_2d = sorted(array_2d, key=lambda row: row[0][1])
            return reordered_array_2d
        
        # Main execution
        sorted_flat_list = flatten_and_sort(array_2d_in_2d)
        grouped_elements = group_by_y_range(sorted_flat_list, group_range)
        new_array_2d_in_2d = ensure_8x8_array(grouped_elements)
        # Ordering each row of new_array_2d_in_2d by "x" value
        ordered_new_array_2d_in_2d = order_rows_by_x(new_array_2d_in_2d)
        # Reordering rows based on the first element's "y" value
        reordered_new_array_2d_in_2d = reorder_rows_by_first_y(ordered_new_array_2d_in_2d)

        return reordered_new_array_2d_in_2d

    
    def _nothing(self, x):
        pass

    def _get_empty_fields(self, cameraImage):
        createTrackBars = False;

        image = cameraImage.copy()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        
        try:
            cv2.getTrackbarPos('param1','Occupancy_canny')
        except:
            createTrackBars = True

        cv2.namedWindow('Occupancy_canny')
        cv2.imshow('Occupancy_canny', blur)

        if(createTrackBars):
            cv2.createTrackbar('param1','Occupancy_canny',1,100, self._nothing) #50
            cv2.setTrackbarMin('param1','Occupancy_canny', 1)
            cv2.createTrackbar('param2','Occupancy_canny',1,100, self._nothing) #10
            cv2.setTrackbarMin('param2','Occupancy_canny', 1)

        occupancy_canny_param1 = cv2.getTrackbarPos('param1','Occupancy_canny')
        occupancy_canny_param2 = cv2.getTrackbarPos('param2','Occupancy_canny')

        while 1:

            if(createTrackBars == True):
                cv2.setTrackbarPos('param1','Occupancy_canny', occupancy_canny_param1)
                cv2.setTrackbarPos('param2','Occupancy_canny', occupancy_canny_param2)

            edges = cv2.Canny(blur,occupancy_canny_param1,occupancy_canny_param2,apertureSize=3)
            cv2.imshow("Occupancy_canny", edges)

            imagem = cv2.bitwise_not(edges)
            # cv2.imshow("Occupancy_imagem", imagem)

            kernel = np.ones((30,30),np.uint8)
            closing = cv2.morphologyEx(imagem, cv2.MORPH_OPEN, kernel)
            cv2.imshow("Occupancy_closing", closing)

            contours = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]

            # print("Occupancy: " + str(len(contours)))
            result = image.copy()
            for c in contours:
                cv2.drawContours(result, [c], -1, (0, 255, 0), 2)

            cv2.imshow("Empty Fields", result)
            # cv2.imshow('Occupancy_result', result)

            if(createTrackBars == False or len(contours) == self.numberOfEmptyFields):
                # print("Occupancy: " + str(len(contours)))
                break
            
            occupancy_canny_param1 += 1
            if(occupancy_canny_param1 == 100):
                occupancy_canny_param1 = 1
                occupancy_canny_param2 += 1

            if(occupancy_canny_param2 == 100):
                occupancy_canny_param1 = 1
                occupancy_canny_param2 = 1
                break

        return contours
    
    def _get_board_from_image(self, cameraImage, emptyFieldsContours):
        board = np.empty((8, 8), dtype=object)
        board.fill(0)

        new_image = cameraImage.copy()
        position = 0;
        for row in range(len(self.gameBoardFieldsContours)):
            for col in range(len(self.gameBoardFieldsContours[row])):
                gameBoardFieldsContours = self.gameBoardFieldsContours[row][col]
                if len(gameBoardFieldsContours) != 4:
                    continue
                isPointInEmptyField = False

                for emptyFieldContour in emptyFieldsContours:
                    x, y, w, h = cv2.boundingRect(emptyFieldContour)
                    point_x = self._get_center_of_field(x, w)
                    point_y = self._get_center_of_field(y, h)

                    isPointInEmptyField = self._is_point_in_field(gameBoardFieldsContours, point_x, point_y)
                    if isPointInEmptyField:
                        break
                
                x, y, w, h = gameBoardFieldsContours
                point_x = self._get_center_of_field(x, w)
                point_y = self._get_center_of_field(y, h)

                if not isPointInEmptyField:
                    isFieldBlack = self._is_field_black(new_image, gameBoardFieldsContours)
                    if isFieldBlack:
                        board[row][col] = 2
                    else:
                        board[row][col] = 1

                else:
                    board[row][col] = 0

                if isPointInEmptyField:
                    # cv2.putText(new_image, str(row) + ", " + str(col), (int(point_x)-10, int(point_y)+5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (246,255,12), 2)
                    cv2.putText(new_image, str(position), (int(point_x)-10, int(point_y)+5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (246,255,12), 2)
                else:
                    # cv2.putText(new_image, str(row) + ", " + str(col), (int(point_x)-10, int(point_y)+5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                    cv2.putText(new_image, str(position), (int(point_x)-10, int(point_y)+5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

                cv2.imshow("gameboard", new_image)
                position = position + 1

    
        new_image = cv2.circle(new_image, (10, 10), radius=10, color=(0, 255, 255), thickness=-1)
                               
        cv2.imshow("gameboard", new_image)

        return board

    def _is_point_in_field(self, rectangle, point_x, point_y):
        rectangle_top_left_x, rectangle_top_left_y, width, height = rectangle
        rectangle_bottom_right_x = rectangle_top_left_x + width
        rectangle_bottom_right_y = rectangle_top_left_y + height

        if rectangle_top_left_x <= point_x <= rectangle_bottom_right_x and rectangle_top_left_y <= point_y <= rectangle_bottom_right_y:
            return True
        else:
            return False

    def _get_center_of_field(self, point, lenght):
        return point + (lenght / 2)

    def _is_field_black(self, img, rectangle, threshold=80):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

        top_left_x, top_left_y, width, height = rectangle
        top_left_x += 5
        top_left_y += 5
        width -= 5
        height -= 5
        # Slice the rectangle from the image
        rectangle = thresh[top_left_y:top_left_y+height, top_left_x:top_left_x+width]
        
        # Calculate the mean value in the rectangle
        mean_value = np.mean(rectangle)

        # Compare the mean value to the threshold
        if mean_value < threshold:
            return True
        else:
            return False
        

    def _is_hand_above_image(self, emptyFieldsContours, allContours):
        # print("HAND")
        # print("Hand: " + str(len(allContours)) + " - " + str(64))
        if(len(allContours) != 64 or len(emptyFieldsContours) != self.numberOfEmptyFields):
            return True
        return False
    
    def _find_contours(self, image):
        createTrackBars = False;

    
        try:
            cv2.getTrackbarPos('param1','all_rectangles')
        except:
            createTrackBars = True

        cv2.namedWindow('all_rectangles')
        cv2.imshow('all_rectangles', image)

        if(createTrackBars):
            cv2.createTrackbar('param1','all_rectangles',1,255, self._nothing) #50
            cv2.setTrackbarMin('param1','all_rectangles', 1)
            cv2.setTrackbarPos('param1','all_rectangles', self.param1ForGetAllContours)

        self.param1ForGetAllContours = cv2.getTrackbarPos('param1','all_rectangles')

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # Apply edge detection
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

         # Detect lines using the Hough Line Transform
        lines = cv2.HoughLines(edges, 1, np.pi / 180, self.param1ForGetAllContours)

        # Create a copy of the original image to draw lines on
        black_image = image.copy()
        black_image = np.zeros_like(black_image)

        # Draw the lines on the image
        if lines is not None:
            for rho, theta in lines[:, 0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * a)
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * a)

                cv2.line(black_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

        gray = cv2.cvtColor(black_image, cv2.COLOR_RGB2GRAY)

        # Find contours
        contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Filter for rectangles
        rectangles = []
        for cnt in contours:
            # Get convex hull
            hull = cv2.convexHull(cnt)
            
            # Get approximate polygon
            epsilon = 0.02 * cv2.arcLength(hull, True)
            approx = cv2.approxPolyDP(hull, epsilon, True)
            
            # Check if it is a rectangle
            if len(approx) == 4:
                rectangles.append(approx)


        # Sort the contours by area and keep the largest one
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        new_image = image.copy()
        correct_result = 0
        new_contours = []
        for position in range(1, len(contours)):

            x,y,w,h = cv2.boundingRect(contours[position])
        
            if(w > 20 and h > 20):
                new_contours.append(contours[position])
                correct_result += 1
                cv2.rectangle(new_image, (x, y), (x + w, y + h), (36,255,12), 1)

        cv2.imshow('all_rectangles', new_image)

        return new_contours

    def get_board(self, cameraImage, game):
        cameraImage = self._trim_image(cameraImage, self.bounderies)
        cv2.imshow("boardCamera", cameraImage)
        
        self.set_number_of_empty_fields(game)

        emptyFieldsContours = self._get_empty_fields(cameraImage)
        # allContours = self._find_contours(cameraImage)
        # isHandAboveBoard = self._is_hand_above_image(emptyFieldsContours, allContours)
        # if(isHandAboveBoard):
        #     print("RUKA")
        #     return True, None
        # else:
        #     return False, self._get_board_from_image(cameraImage, emptyFieldsContours)

        return self._get_board_from_image(cameraImage, emptyFieldsContours)
        
    def set_number_of_empty_fields(self, game):
        self.numberOfEmptyFields = 64 - game.board.black_left - game.board.white_left
        