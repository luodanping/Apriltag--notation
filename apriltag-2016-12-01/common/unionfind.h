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
//并查集=》解决动态连通性问题的一类非常高效的数据结构：https://blog.csdn.net/guoziqing506/article/details/78752557
#ifndef _UNIONFIND_H
#define _UNIONFIND_H

#include <stdint.h>
#include <stdlib.h>

typedef struct unionfind unionfind_t;

struct unionfind //unionfind结构体对象
{
    uint32_t maxid; //应该表示数据的最大个数
    struct ufrec *data;
};

struct ufrec
{
    // the parent of this node. If a node's parent is its own index,
    // then it is a root. //节点的父节，如果为自己索引，则其为根节点
    uint32_t parent; //根节点表示元素所在的集合

    // for the root of a connected component, the number of components
    // connected to it. For intermediate values, it's not meaningful.
    uint32_t size; //当连接部分的根时，与之相连的单元数量，对于中间的就没有意义
};

static inline unionfind_t *unionfind_create(uint32_t maxid) //创建对象
{
    unionfind_t *uf = (unionfind_t*) calloc(1, sizeof(unionfind_t));
    uf->maxid = maxid;
    uf->data = (struct ufrec*) malloc((maxid+1) * sizeof(struct ufrec)); //产生maxid+1个ufrec
    for (int i = 0; i <= maxid; i++) { //每个data都是一个ufrec对象
        uf->data[i].size = 1; //每个的个数初始化为1
        uf->data[i].parent = i;//父节点初始化为自己
    }
    return uf;//返回这个对象
}

static inline void unionfind_destroy(unionfind_t *uf) //删除所有数据及对象本身
{
    free(uf->data);
    free(uf);
}

/*
static inline uint32_t unionfind_get_representative(unionfind_t *uf, uint32_t id)
{
    // base case: a node is its own parent
    if (uf->data[id].parent == id)
        return id;

    // otherwise, recurse
    uint32_t root = unionfind_get_representative(uf, uf->data[id].parent);

    // short circuit the path. [XXX This write prevents tail recursion]
    uf->data[id].parent = root;

    return root;
}
*/

// this one seems to be every-so-slightly faster than the recursive
// version above.==>这个似乎比上面递归版本要快
static inline uint32_t unionfind_get_representative(unionfind_t *uf, uint32_t id) //找到父节点
{//找到相关节点
    uint32_t root = id;

    // chase down the root
    while (uf->data[root].parent != root) { //如果不是父节点，则查找其父节点
        root = uf->data[root].parent;
    }

    // go back and collapse the tree.
    //
    // XXX: on some of our workloads that have very shallow trees
    // (e.g. image segmentation), we are actually faster not doing
    // this...
    while (uf->data[id].parent != root) { //遍历直到该id的parent是root。
        uint32_t tmp = uf->data[id].parent;//当前id的父节点给tmp
        uf->data[id].parent = root;//当前id的父节点设置为root
        id = tmp;//当前节点的父节点，执行同样操作（归类）
    }

    return root;
}

static inline uint32_t unionfind_get_set_size(unionfind_t *uf, uint32_t id)
{
    uint32_t repid = unionfind_get_representative(uf, id); //找到父节点
    return uf->data[repid].size;//返回size（父节点的size才有意义）
}

static inline uint32_t unionfind_connect(unionfind_t *uf, uint32_t aid, uint32_t bid)
{//增加a，b的连通关系(处理两种情况1.已经连通 2.没有连通)
    uint32_t aroot = unionfind_get_representative(uf, aid); //找到a的父节点
    uint32_t broot = unionfind_get_representative(uf, bid);//找到b的父节点

    if (aroot == broot)//如果两个的父节点是一样的，表示他们是联通的
        return aroot;//连通的则返回父节点即可

    // we don't perform "union by rank", but we perform a similar
    // operation (but probably without the same asymptotic(adj渐进的，渐近线的) guarantee):
    // We join trees based on the number of *elements* (as opposed to
    // rank) contained within each tree. I.e., we use size as a proxy
    // for rank.  In my testing, it's often *faster* to use size than
    // rank, perhaps because the rank of the tree isn't that critical
    // if there are very few nodes in it.
    uint32_t asize = uf->data[aroot].size; //a的size
    uint32_t bsize = uf->data[broot].size;//b的size

    // optimization idea: We could shortcut some or all of the tree
    // that is grafted onto the other tree. Pro: those nodes were just
    // read and so are probably in cache. Con: it might end up being
    // wasted effort -- the tree might be grafted(移植，嫁接) onto another tree in
    // a moment!
    if (asize > bsize) { //a的size比b的大
        uf->data[broot].parent = aroot;//b的parent设置为a的
        uf->data[aroot].size += bsize;//尺寸增加
        return aroot;
    } else {//反之，同
        uf->data[aroot].parent = broot;
        uf->data[broot].size += asize;
        return broot;
    }
}
#endif
