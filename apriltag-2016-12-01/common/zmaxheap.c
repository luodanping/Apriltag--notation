/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <stdint.h>

#include "zmaxheap.h"

//                 0
//         1               2
//      3     4        5       6===》size/2-1
//     7 8   9 10    11 12   13 14（这些id都是叶子节点，此行之上的都可作为父节点，且id连续）
//
// Children of node i:  2*i+1, 2*i+2 //node i的子节点对应的node号码
// Parent of node i: (i-1) / 2       //根据上面公式，可以反算出node i的父节点号码
//
// Heap property: a parent is greater than (or equal to) its children.//父节点的值大于或等于所有子节点中的值

#define MIN_CAPACITY 16

struct zmaxheap //堆结构
{
    size_t el_sz; //元素大小-指的是char× data中每个字符串的长度(从swap_heap()函数推测的)

    int size;//当前存的数量
    int alloc; //分配的容量

    float *values; //这个values有什么用？
    char *data;

    void (*swap)(zmaxheap_t *heap, int a, int b);//结构体中元素使用函数指针实现成员函数功能
};

static inline void swap_default(zmaxheap_t *heap, int a, int b) //交换ab处的值
{
    float t = heap->values[a]; //values里的值
    heap->values[a] = heap->values[b];
    heap->values[b] = t;

    char tmp[heap->el_sz];//交换两处string，没处的大小为heap->el_sz
    memcpy(tmp, &heap->data[a*heap->el_sz], heap->el_sz);
    memcpy(&heap->data[a*heap->el_sz], &heap->data[b*heap->el_sz], heap->el_sz);
    memcpy(&heap->data[b*heap->el_sz], tmp, heap->el_sz);
}

static inline void swap_pointer(zmaxheap_t *heap, int a, int b)//交换ab处的值
{
    float t = heap->values[a];
    heap->values[a] = heap->values[b];
    heap->values[b] = t;

    void **pp = (void**) heap->data;//使用二重指针来实现交换：data里面存的内容变成是指针？
    void *tmp = pp[a];
    pp[a] = pp[b];
    pp[b] = tmp;
}


zmaxheap_t *zmaxheap_create(size_t el_sz) //创建heap对象
{
    zmaxheap_t *heap = calloc(1, sizeof(zmaxheap_t));
    heap->el_sz = el_sz; //设置char中

    heap->swap = swap_default;//设置成员函数的指针

    if (el_sz == sizeof(void*)) //假如data中元素尺寸为指针，则成员函数变成操作指针
        heap->swap = swap_pointer;

    return heap;
}

void zmaxheap_destroy(zmaxheap_t *heap) //对应zmaxheap_creat()，把calloc等分配的清空
{
    free(heap->values);
    free(heap->data);
    memset(heap, 0, sizeof(zmaxheap_t));
    free(heap);
}

int zmaxheap_size(zmaxheap_t *heap) //返回对象的size
{
    return heap->size;
}

void zmaxheap_ensure_capacity(zmaxheap_t *heap, int capacity)//保证容量
{
    if (heap->alloc >= capacity) //分配的大于需要的容量，则直接返回
        return;

    int newcap = heap->alloc;

    while (newcap < capacity) { //如果分配的比需要的小，则调整
        if (newcap < MIN_CAPACITY) { //假如比最小容量小，则设置为最小容量
            newcap = MIN_CAPACITY;
            continue;
        }

        newcap *= 2; //两倍，跟zarray中的一样
    }

    heap->values = realloc(heap->values, newcap * sizeof(float));//将参数1指向的数据块，移到新的位置
    heap->data = realloc(heap->data, newcap * heap->el_sz);
    heap->alloc = newcap; //使用新的容量
}

void zmaxheap_add(zmaxheap_t *heap, void *p, float v) //增加一个元素
{
//isfinite==>math.h 宏
    assert (isfinite(v) && "zmaxheap_add: Trying to add non-finite number to heap.  NaN's prohibited, could allow INF with testing");
    zmaxheap_ensure_capacity(heap, heap->size + 1);//保证容量

    int idx = heap->size;//最后元素后的地址

    heap->values[idx] = v;//添加值
    memcpy(&heap->data[idx*heap->el_sz], p, heap->el_sz);//添加字符串

    heap->size++;//添加后size+1

    while (idx > 0) { //整理大小

        int parent = (idx - 1) / 2; //求取idx对应的parent

        // we're done!
        if (heap->values[parent] >= v)  //满足parent大于children的值
            break;

        // else, swap and recurse upwards. //否则交换idx和parent
        heap->swap(heap, idx, parent);
        idx = parent;
    }
}

void zmaxheap_vmap(zmaxheap_t *heap, void (*f)())  //heap中的元素，通过p进行映射，输出应该通过传入函数指针f中的变量指针实现
{
    assert(heap != NULL);
    assert(f != NULL);
    assert(heap->el_sz == sizeof(void*));

    for (int idx = 0; idx < heap->size; idx++) { //遍历heap中所有的元素
        void *p = NULL;
        memcpy(&p, &heap->data[idx*heap->el_sz], heap->el_sz);
        if (p == NULL) {
            printf("Warning: zmaxheap_vmap item %d is NULL\n", idx);
            fflush(stdout);//刷新流的输出缓冲区
        }
        f(p);//将p传入函数f中，进行map映射
    }
}

// Removes the item in the heap at the given index.  Returns 1 if the
// item existed. 0 Indicates an invalid idx (heap is smaller than
// idx). This is mostly intended to be used by zmaxheap_remove_max.//主要给zmaxheap_remove_max使用
int zmaxheap_remove_index(zmaxheap_t *heap, int idx, void *p, float *v)//删除给定indx处的元素
{
    if (idx >= heap->size) //索引idx大于heap的尺寸
        return 0;

    // copy out the requested element from the heap.//从heap中复制出该索引处的值和data
    if (v != NULL)
        *v = heap->values[idx];
    if (p != NULL)
        memcpy(p, &heap->data[idx*heap->el_sz], heap->el_sz);

    heap->size--; //size-1

    // If this element is already the last one, then there's nothing
    // for us to do. //加入元素为最后一个，就不用其他操作了
    if (idx == heap->size) 
        return 1;

    // copy last element to first element. (which probably upsets
    // the heap property). //把idx处的删除相当于把idx的值用最后一个元素的值替代，然后再对整个heap进行修复使之满足heap的属性要求
    heap->values[idx] = heap->values[heap->size];
    memcpy(&heap->data[idx*heap->el_sz], &heap->data[heap->el_sz * heap->size], heap->el_sz);

    // now fix the heap. Note, as we descend, we're "pushing down"
    // the same node the entire time. Thus, while the index of the
    // parent might change, the parent_score doesn't.
    int parent = idx;  //将idx处做为parent的，向下遍历
    float parent_score = heap->values[idx];

    // descend, fixing the heap.
    while (parent < heap->size) {

        int left = 2*parent + 1; //根据上面的公式可以计算左右节点的索引
        int right = left + 1;

//            assert(parent_score == heap->values[parent]);

        float left_score = (left < heap->size) ? heap->values[left] : -INFINITY;
        float right_score = (right < heap->size) ? heap->values[right] : -INFINITY;

        // put the biggest of (parent, left, right) as the parent. //将父子节点的值进行比较，取最大的最为父

        // already okay?
        if (parent_score >= left_score && parent_score >= right_score)
            break;

        // if we got here, then one of the children is bigger than the parent.
        if (left_score >= right_score) {
            assert(left < heap->size);
            heap->swap(heap, parent, left);
            parent = left;
        } else {
            // right_score can't be less than left_score if right_score is -INFINITY.
            assert(right < heap->size);
            heap->swap(heap, parent, right);
            parent = right;
        }
    }

    return 1;
}

int zmaxheap_remove_max(zmaxheap_t *heap, void *p, float *v)//除去最大的值
{
    return zmaxheap_remove_index(heap, 0, p, v);//0处的值是最大的，索引=0
}

void zmaxheap_iterator_init(zmaxheap_t *heap, zmaxheap_iterator_t *it) //用heap去初始化iterator
{
    memset(it, 0, sizeof(zmaxheap_iterator_t));
    it->heap = heap; //将heap对象赋值给it的成员
    it->in = 0;
    it->out = 0;
}

int zmaxheap_iterator_next(zmaxheap_iterator_t *it, void *p, float *v) //从上到下从左到右遍历heap
{//输出当前it->in处的值，并且将it->in加1，指向下一个值
//判断out==it,假如相等，则什么不做。不相等，则说明调用过zmaxheap_iterator_remove，则此时out=in-1（第一次，如果上面函数再次调用，则依次会in-2，in-3,...）
    zmaxheap_t *heap = it->heap;//heap对象，保存数据

    if (it->in >= zmaxheap_size(heap)) //假如in达到最大的元素个数，则返回0，否则返回1表示可以继续遍历
        return 0;

    *v = heap->values[it->in]; //值保存在v中
    memcpy(p, &heap->data[it->in*heap->el_sz], heap->el_sz);//data保存在p中，记号1

    if (it->in != it->out) {  //这个地方的操作是什么意思？==参见本文件中函数zmaxheap_iterator_remove
        heap->values[it->out] = heap->values[it->in]; //如果调用过zmaxheap_iterator_remove则out减1，当前in处的值取覆盖前一处的值。在zmaxheap_iterator_finish函数中，就会进行过相关处理，得到有删除元素后的heap
        memcpy(&heap->data[it->out*heap->el_sz], &heap->data[it->in*heap->el_sz], heap->el_sz);
    }

    it->in++;
    it->out++;
    return 1;
}

int zmaxheap_iterator_next_volatile(zmaxheap_iterator_t *it, void *p, float *v)//功能同zmaxheap_iterator_next
{
    zmaxheap_t *heap = it->heap;

    if (it->in >= zmaxheap_size(heap))
        return 0;

    *v = heap->values[it->in];
    *((void**) p) = &heap->data[it->in*heap->el_sz]; //p直接使用data中的地址

    if (it->in != it->out) {
        heap->values[it->out] = heap->values[it->in];
        memcpy(&heap->data[it->out*heap->el_sz], &heap->data[it->in*heap->el_sz], heap->el_sz);
    }

    it->in++;
    it->out++;
    return 1;
}

void zmaxheap_iterator_remove(zmaxheap_iterator_t *it)//移除当前元素(这个当前指的是哪个当前？)
{
    it->out--; //out-1（感觉这个remove是将heap中的element进行清除，并且还需要给iterator搭配使用）
}

static void maxheapify(zmaxheap_t *heap, int parent)//用于在迭代器完成的时候，重新构建heap的属性
{
    int left = 2*parent + 1; //父节点左边子节点
    int right = 2*parent + 2;//父节点右边子节点

    int betterchild = parent;

    if (left < heap->size && heap->values[left] > heap->values[betterchild])
        betterchild = left;  //左边大于最好的，则最好的为左边
    if (right < heap->size && heap->values[right] > heap->values[betterchild])
        betterchild = right;//右边的大于最好的，则最好的为右边

    if (betterchild != parent) { //假如最好的不是父的
        heap->swap(heap, parent, betterchild); //则最好的与父的交换，交换索引对应的值
        return maxheapify(heap, betterchild); //递归需要终止条件，所以并是不递归（因为此处的值与一下比自己小的值进行了交换，所以需要将它再次与自己的子节点进行比较，这样才能保证正确性。没有发生值交换的节点就不会存在这样的情况）
    }
}

#if 0 //won't compile if defined but not used
// test the heap property
static void validate(zmaxheap_t *heap)
{
    for (int parent = 0; parent < heap->size; parent++) {
        int left = 2*parent + 1;
        int right = 2*parent + 2;

        if (left < heap->size) {
            assert(heap->values[parent] > heap->values[left]);
        }

        if (right < heap->size) {
            assert(heap->values[parent] > heap->values[right]);
        }
    }
}
#endif
void zmaxheap_iterator_finish(zmaxheap_iterator_t *it)
{
    // if nothing was removed, no work to do.
    if (it->in == it->out) //in==out则不需要任何处理
        return;

    zmaxheap_t *heap = it->heap;

    heap->size = it->out; //因为除去了东西，则size就会变短

    // restore heap property  //重新构建heap的结构
    for (int i = heap->size/2 - 1; i >= 0; i--) //size/2-1表示最后元素的父节点(跳过所有叶子节点，剩余的都是父节点，且他们id是连续的)
        maxheapify(heap, i);
}

void zmaxheap_test() //测试的吧
{
    int cap = 10000;
    int sz = 0;
    int32_t *vals = calloc(sizeof(int32_t), cap);

    zmaxheap_t *heap = zmaxheap_create(sizeof(int32_t));

    int maxsz = 0;
    int zcnt = 0;

    for (int iter = 0; iter < 5000000; iter++) {
        assert(sz == heap->size);

        if ((random() & 1) == 0 && sz < cap) {
            // add a value
            int32_t v = (int32_t) (random() / 1000);
            float fv = v;
            assert(v == fv);

            vals[sz] = v;
            zmaxheap_add(heap, &v, fv);
            sz++;

//            printf("add %d %f\n", v, fv);
        } else {
            // remove a value
            int maxv = -1, maxi = -1;

            for (int i = 0; i < sz; i++) {
                if (vals[i] > maxv) {
                    maxv = vals[i];
                    maxi = i;
                }
            }


            int32_t outv;
            float outfv;
            int res = zmaxheap_remove_max(heap, &outv, &outfv);
            if (sz == 0) {
                assert(res == 0);
            } else {
//                printf("%d %d %d %f\n", sz, maxv, outv, outfv);
                assert(outv == outfv);
                assert(maxv == outv);

                // shuffle erase the maximum from our list.
                vals[maxi] = vals[sz - 1];
                sz--;
            }
        }

        if (sz > maxsz)
            maxsz = sz;

        if (maxsz > 0 && sz == 0)
            zcnt++;
    }

    printf("max size: %d, zcount %d\n", maxsz, zcnt);
    free (vals);
}
