def create_nodes_Flow1D(femmesh):
    # nodes
    femmesh.addNode(0, 0, 50, 1)
    femmesh.addNode(0, 0, -50, 2)
    femmesh.addNode(0, 0, -4300, 3)
    femmesh.addNode(4950, 0, -4300, 4)
    femmesh.addNode(5000, 0, -4300, 5)
    femmesh.addNode(8535.53, 0, -7835.53, 6)
    femmesh.addNode(8569.88, 0, -7870.88, 7)
    femmesh.addNode(12105.41, 0, -11406.41, 8)
    femmesh.addNode(12140.76, 0, -11441.76, 9)
    femmesh.addNode(13908.53, 0, -13209.53, 10)
    femmesh.addNode(13943.88, 0, -13244.88, 11)
    femmesh.addNode(15046.97, 0, -14347.97, 12)
    femmesh.addNode(15046.97, 0, -7947.97, 13)
    femmesh.addNode(15046.97, 0, -7847.97, 14)
    femmesh.addNode(0, 0, 0, 15)
    femmesh.addNode(0, 0, -2175, 16)
    femmesh.addNode(2475, 0, -4300, 17)
    femmesh.addNode(4975, 0, -4300, 18)
    femmesh.addNode(6767.765, 0, -6067.765, 19)
    femmesh.addNode(8552.705, 0, -7853.205, 20)
    femmesh.addNode(10337.645, 0, -9638.645, 21)
    femmesh.addNode(12123.085, 0, -11424.085, 22)
    femmesh.addNode(13024.645, 0, -12325.645, 23)
    femmesh.addNode(13926.205, 0, -13227.205, 24)
    femmesh.addNode(14495.425, 0, -13796.425, 25)
    femmesh.addNode(15046.97, 0, -11147.97, 26)
    femmesh.addNode(15046.97, 0, -7897.97, 27)
    femmesh.addNode(15046.97, 0, -7897.97, 28)
    return True


def create_elements_Flow1D(femmesh):
    # elements
    femmesh.addEdge([1, 2, 15], 1)
    femmesh.addEdge([2, 3, 16], 2)
    femmesh.addEdge([3, 4, 17], 3)
    femmesh.addEdge([4, 5, 18], 4)
    femmesh.addEdge([5, 6, 19], 5)
    femmesh.addEdge([6, 7, 20], 6)
    femmesh.addEdge([7, 8, 21], 7)
    femmesh.addEdge([8, 9, 22], 8)
    femmesh.addEdge([9, 10, 23], 9)
    femmesh.addEdge([10, 11, 24], 10)
    femmesh.addEdge([11, 12, 25], 11)
    femmesh.addEdge([12, 13, 26], 12)
    femmesh.addEdge([13, 28, 27], 13)
    return True
