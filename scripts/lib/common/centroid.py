import matplotlib.pyplot as plt
import numpy as np

def calculate_centroid(points):
    sx = sy= sz = sL = 0
    for i in range(len(points)):
        x0, y0, z0 = points[i - 1]     # in Python points[-1] is last element of points
        x1, y1, z1 = points[i]
        L = ((x1 - x0)**2 + (y1 - y0)**2 + (z1-z0)**2) ** 0.5
        sx += (x0 + x1)/2 * L
        sy += (y0 + y1)/2 * L
        sz += (z0 + z1)/2 * L
        sL += L
        
    centroid_x = sx / sL
    centroid_y = sy / sL
    centroid_z = sz / sL

    print('cent x = %f, cent y = %f, cent z = %f'%(centroid_x, centroid_y, centroid_z))

    # TODO: 계산하는 공식 추가하기
    return np.array([centroid_x,centroid_y, centroid_z])

def sorted_points(points):
    xs = []
    xy = []
    for i in points:
        xs.append(i[0])
        xy.append(i[1])

    harf_x = (max(xs) + min(xs)) / 2 
    
    # 겹치는 x값이 있는지 확인
    if harf_x in xs:
        harf_x = harf_x + 0.001
    
    y_right = []
    y_left = []

    #harf_x보다 x값이 작은 좌표집합과 큰 좌표집합으로 분리
    for i in points:
        if i[0] < harf_x:
            y_left.append(i)
        else: 
            y_right.append(i)

    y_left.sort(key=lambda x : x[1] , reverse = False)
    y_right.sort(key=lambda x : x[1] , reverse = True)

    return np.array(y_left + y_right)

def test_calculate_centroid():
    pt1 = [5, 0]
    pt2 = [10,10]
    pt3 = [0, 0]
    pt4 = [10,0]
    # pt4 = [10,5]
    pt5 = [0,10]
    pt6 = [0, 5]
    pt7 = [0, 8]
    # pt6 = [4, 7]
    points = np.array([pt1, pt2, pt3, pt4, pt5, pt6, pt7])

    # 포인트정렬
    sortedPoints = sorted_points(points)

    # 입력값
    points_x = sortedPoints[:,0]
    points_y = sortedPoints[:,1]

    # 출력값
    centroid = calculate_centroid(sortedPoints)
    centroid_x = centroid[0]
    centroid_y = centroid[1]

    # 원본 데이터 그리는 방법 (1)
    plt.plot(points_x, points_y, 'bo') # blue, marker = o

    # 원본 데이터 그리는 방법 (2)
    # 도형의 선분까지 표현하려면, pt1을 추가해주고, 선을 그리도록 하면 된다.
    points_for_draw = np.vstack([sortedPoints, pt1])
    points_for_draw_x = points_for_draw[:,0]
    points_for_draw_y = points_for_draw[:,1]
    plt.plot(points_for_draw_x, points_for_draw_y, 'g-o') # blue, marker = o, line style = -

    # 출력값 그리기
    plt.plot(centroid_x, centroid_y, 'rx') # red, marker = x
    plt.draw()

    plt.show()

if __name__ == u'__main__':
    test_calculate_centroid()