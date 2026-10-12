import numpy as np

class MinHeap:
    # Binary min-heap of Elements ordered by sortKey(). It grows as needed; there is no size limit.
    def __init__(self): 
        self.size = 0
        self.Heap = []
        self.FRONT = 0

    def parent(self, pos): 
        return (pos + 1)//2 - 1
    
    def leftChild(self, pos): 
        return 2 * (pos + 1) - 1

    def rightChild(self, pos):
        return 2 * (pos + 1)
    
    def value(self, pos): 
        return self.Heap[pos].sortKey()
    
    def leftChildExists(self, pos): 
        return (pos + 1) * 2 - 1 < self.size
    
    def rightChildExists(self, pos): 
        return (pos + 1) * 2 < self.size
    
    def swap(self, fpos, spos): 
        self.Heap[fpos], self.Heap[spos] = self.Heap[spos], self.Heap[fpos] 

    def minHeapifyUp(self, pos, forced):
        left_child_exists = self.leftChildExists(pos)
        right_child_exists = self.rightChildExists(pos)
        if right_child_exists and self.value(pos) > self.value(self.rightChild(pos)):
            self.swap(pos, self.rightChild(pos))
            if not forced:
                self.minHeapifyUp(self.rightChild(pos), forced)
        if left_child_exists and self.value(pos) > self.value(self.leftChild(pos)):
            self.swap(pos, self.leftChild(pos))
            if not forced:
                self.minHeapifyUp(self.leftChild(pos), forced)
        if forced:
            self.minHeapifyUp(self.leftChild(pos), forced)
            self.minHeapifyUp(self.rightChild(pos), forced)


    def minHeapifyDown(self, pos):
        left_child_exists = self.leftChildExists(pos)
        right_child_exists = self.rightChildExists(pos)
        if right_child_exists and self.value(pos) > self.value(self.rightChild(pos)):
            self.swap(pos, self.rightChild(pos))
            if(pos != 0):
                self.minHeapifyDown(self.parent(pos))
        if left_child_exists and self.value(pos) > self.value(self.leftChild(pos)):
            self.swap(pos, self.leftChild(pos))
            if(pos != 0):
                self.minHeapifyDown(self.parent(pos))
            
    def insert(self, element):
        if self.size == len(self.Heap):
            self.Heap.append(element)
        else:
            self.Heap[self.size] = element
        self.size += 1

        parent_pos = self.parent(self.size - 1)
        if(parent_pos >= 0):
            self.minHeapifyDown(parent_pos)

    def batchInsert(self, elements):
        for element in elements:
            self.insert(element)

    def minHeap(self): 
        for pos in range(self.size//2 - 1, 0, -1): 
            self.minHeapifyUp(pos, True) 

    def remove(self): 
        popped = self.Heap[0]
        self.Heap[0] = self.Heap[self.size - 1] 
        self.size -= 1
        if(self.size != 0):
            self.minHeapifyUp(0, False) 
        return popped
        
class Element:
    def __init__(self, x, y, other = None, stepWeight = 1, parent = None): 
        self.x = x 
        self.y = y

        if(other is None):
            self.value = 0
        else:
            self.value = np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

        self.stepWeight = stepWeight

        if(parent is None):
            self.steps = 0
            self.parent = None
        else:
            self.parent = parent
            self.steps = parent.steps + self.stepWeight

    def __eq__(self, other):
        if isinstance(other, Element):
            return other.x == self.x and other.y == self.y
        return False

    def toPoint(self):
        return [self.x, self.y]

    def addWeight(self, weight):
        self.stepWeight += weight
        self.steps += weight

    def createSuccessors(self, maxX, maxY, other, stepWieght = 1):
        successors = []
        if(self.x > 0):
            successors.append(Element(self.x - 1, self.y, other, stepWieght, self))
        
        if(self.y > 0):
            successors.append(Element(self.x, self.y - 1, other, stepWieght, self))
        
        if(self.x < maxX - 1):
            successors.append(Element(self.x + 1, self.y, other, stepWieght, self))
        
        if(self.y < maxY - 1):
            successors.append(Element(self.x, self.y + 1, other, stepWieght, self))

        return successors

    def getValue(self):
        return self.value + self.steps

    def sortKey(self):
        # A* cost, ties broken lexicographically: smaller x first, then smaller y.
        return (self.getValue(), self.x, self.y)
    
def Flatten2DElement(value):
    return f'({value[0]},  {value[1]})'

def Flatten3DElement(value):
    return f'({value[0]}, {value[1]}, {value[2]})'
    
def ContainsFor2D(value, array):
    flattenElement = lambda e: f'({e[0]},  {e[1]})'
    newArray = map(flattenElement, array)
    newValue = flattenElement(value)
    return newValue in newArray