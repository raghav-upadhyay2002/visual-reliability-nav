import sys
import cv2
import numpy as np


def _to_gray(image):
    '''Accept either a grayscale (2D) or BGR/BGRA (3D) array and return grayscale.'''
    if image.ndim == 2:
        return image
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


# Pipleline A: Thresholding + Morphological closing + Contour detection
def detect_walls_countours(image, min_wall_area=50):
    '''
    Detect walls as filled blobs using thresholding + closing + contour.
    image: grayscale or BGR/BGRA array (e.g. straight from camera.getImage()).
    returns: walls and intermediate images.
    '''
    img = _to_gray(image)
    output = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    #1. Thresholding
    # THRESH_BINARY_INV: black walls on white background
    _, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    #2 Morphological closing to fill holes in walls
    kernel = np.ones((5, 5), np.uint8)
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

    #3. Contour detection
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    walls = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_wall_area:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        walls.append({"contour": cnt, "bbox": (x, y, w, h), "area": area})

        cv2.drawContours(output, [cnt], -1, (0, 255, 0), 2)
        cv2.rectangle(output, (x, y), (x + w, y + h), (0, 0, 255), 1)

    return walls, {"threshold": thresh, "closed": closed, "result": output}


# Pipeline B: Canny edge detection + Morphological closing + Hough line detection
def detect_walls_lines(image, canny_low=50, canny_high=150, hough_threshold=50, min_line_length=30, max_line_gap=10):
    '''
    Detect walls as lines using Canny edge detection + closing + Hough line detection.
    image: grayscale or BGR/BGRA array (e.g. straight from camera.getImage()).
    returns: walls and intermediate images.
    '''
    img = _to_gray(image)
    output = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    #1 Blur to reduce noise
    blurred = cv2.GaussianBlur(img, (5, 5), 0)

    #2 Canny edge detection
    edges = cv2.Canny(blurred, canny_low, canny_high)

    #3 Morphological closing to fill gaps in edges
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(edges, kernel, iterations=1)

    #4 Hough line detection
    raw_lines = cv2.HoughLinesP(dilated, 1, np.pi / 180, hough_threshold, minLineLength=min_line_length, maxLineGap=max_line_gap)

    lines = []
    if raw_lines is not None:
        for line in raw_lines:
            x1, y1, x2, y2 = line[0]
            length = np.hypot(x2 - x1, y2 - y1)
            lines.append({'p1': (x1, y1), 'p2': (x2, y2), 'length': length})
            cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 2)

    return lines, {'edges': edges, 'dilated': dilated, 'result': output}


if __name__ == "__main__":
    # Standalone testing on a saved screenshot, e.g.:
    #   python classic_cv.py path/to/frame.png
    if len(sys.argv) < 2:
        print("Usage: python classic_cv.py <image_path>")
        sys.exit(1)

    path = sys.argv[1]
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Could not read image: {path}")
        sys.exit(1)

    walls, debug_a = detect_walls_countours(img)
    print(f"detect_walls_countours: {len(walls)} wall blob(s) found")
    cv2.imshow("Pipeline A - threshold", debug_a["threshold"])
    cv2.imshow("Pipeline A - closed", debug_a["closed"])
    cv2.imshow("Pipeline A - result", debug_a["result"])

    lines, debug_b = detect_walls_lines(img)
    print(f"detect_walls_lines: {len(lines)} line(s) found")
    cv2.imshow("Pipeline B - edges", debug_b["edges"])
    cv2.imshow("Pipeline B - result", debug_b["result"])

    cv2.waitKey(0)
    cv2.destroyAllWindows()
