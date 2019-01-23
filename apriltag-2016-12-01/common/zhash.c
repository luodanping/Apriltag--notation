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
//与thash_impl.h中的内容几乎一直，推测是thash_impl.h中通用操作没能实现完
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "zhash.h"
//cnblog:hash算法总结收集【作者：wanghetao】
//提供了一种快速存取数据的方法，它用一种算法建立键值与真实值之间的对应关系（每个真实值只能有一个键值，但一个键值可以对应多个真实值）
//cnblog：几种哈希算法原理（转发）【USBZPZOJ】
// force a rehash when our capacity is less than this many times the size
#define ZHASH_FACTOR_CRITICAL 2

// When resizing, how much bigger do we want to be? (should be greater than _CRITICAL)
#define ZHASH_FACTOR_REALLOC 4

struct zhash
{
    size_t keysz, valuesz; //key和value的尺寸大小
    int    entrysz; // valid byte (1) + keysz + valuesz //实体的大小=1+key的大小+value的大小（这个1位应该是用来表示这个是否有效valid）

    uint32_t(*hash)(const void *a);//生成hash的函数

    // returns 1 if equal==》判断a跟b是否相同(在使用put等方法的时候，会判断key值的)
    int(*equals)(const void *a, const void *b);

    int size; // # of items in hash table==》元素的个数（已存的）

    char *entries; // each entry of size entrysz; //分配的空间=分配实体个数×每个实体的大小
    int  nentries; // how many entries are allocated? Never 0. //容量==》桶的个数？
};

zhash_t *zhash_create_capacity(size_t keysz, size_t valuesz,
                               uint32_t(*hash)(const void *a), int(*equals)(const void *a, const void*b),
                               int capacity)
{
    assert(hash != NULL); //生成hash值的函数
    assert(equals != NULL);//比较hash值的函数

    // resize...
    int _nentries = ZHASH_FACTOR_REALLOC * capacity; //当重新分配容量时，分配为capacity的ZHASH_FACTOR_REALLOC倍
    if (_nentries < 8)
        _nentries = 8;

    // to a power of 2.
    int nentries = _nentries;
    if ((nentries & (nentries - 1)) != 0) {
        nentries = 8;
        while (nentries < _nentries)
            nentries *= 2;
    }  //容量要是2的指数==
//calloc(分配个数，每个的大小) 参数乘积就是要分配空间的大小  malloc(内存空间大小) 返回的是(void*)
    zhash_t *zh = (zhash_t*) calloc(1, sizeof(zhash_t)); //生成一个hash对象
    zh->keysz = keysz;  
    zh->valuesz = valuesz;
    zh->hash = hash;
    zh->equals = equals;
    zh->nentries = nentries;

    zh->entrysz = 1 + zh->keysz + zh->valuesz; //valid位+key的size+value的size

    zh->entries = calloc(zh->nentries, zh->entrysz); //分配的内存空间
    zh->nentries = nentries; //分配的容量（不一定被占用了）

    return zh;
}

zhash_t *zhash_create(size_t keysz, size_t valuesz,  //创建hash对象
                      uint32_t(*hash)(const void *a), int(*equals)(const void *a, const void *b))
{
    return zhash_create_capacity(keysz, valuesz, hash, equals, 8); //传感capacity=8的hash
}

void zhash_destroy(zhash_t *zh)//删除hash对象
{
    if (zh == NULL)
        return;

    free(zh->entries); //对应zhash_create_capacity中的calloc()函数
    free(zh); //malloc告诉系统要用哪段内存，free告诉系统已经用完不要了
}

int zhash_size(const zhash_t *zh)//返回hash表中的元素的个数
{
    return zh->size;
}

void zhash_clear(zhash_t *zh)//删除所有hash元素
{
    memset(zh->entries, 0, zh->nentries * zh->entrysz); //内存归零
    zh->size = 0;//size设为0，表示没有元素
}

int zhash_get_volatile(const zhash_t *zh, const void *key, void *out_value)//获取key的对应的value
{
    uint32_t code = zh->hash(key); //通过key生成hash值
    uint32_t entry_idx = code & (zh->nentries - 1); //=code%length.看放在length长的buckets的哪个中

    while (zh->entries[entry_idx * zh->entrysz]) { //valid值，1表示有数据正常存储（put函数）==》继续遍历
        void *this_key = &zh->entries[entry_idx * zh->entrysz + 1];//
        if (zh->equals(key, this_key)) {//判断key是否存在，存在则返回值out_value中
            *((void**) out_value) = &zh->entries[entry_idx * zh->entrysz + 1 + zh->keysz];
            return 1;
        }
//参考文献：常见hash算法的原理 【作者：Brenda】
//解决冲突的方法1.线性探测法，冲突后，线性向前找到最近的一个空位置
//方法2.双散列，冲突后，再次使用hash产生一个与散列表桶容量m互质的数c，一次试探
        entry_idx = (entry_idx + 1) & (zh->nentries - 1);//这一步是干啥的？（猜测：上面如果hash值相同，但是key可能不同，所以查找其他的可能存储点，继续进行匹配）
    }//end of while；

    return 0;
}

int zhash_get(const zhash_t *zh, const void *key, void *out_value)
{
    void *tmp;//新的地址（最后指向hash的地址）
    if (zhash_get_volatile(zh, key, &tmp)) {//引用：别名
        memcpy(out_value, tmp, zh->valuesz);//把tmp中的内容复制到out_value中，防止out_value在hash中
        return 1;
    }

    return 0;
}
//文献参考： [CSDN]关于HashMap的一些按位与计算的问题
int zhash_put(zhash_t *zh, const void *key, const void *value, void *oldkey, void *oldvalue)
{//往里面加入key/value pairs： 具体算法可参照网上关于hash get的实现
//有两个问题要解决，1. 元素存储在哪个位置 2 如果key出现hash冲突，如何解决（两个hash算出同一个hash）
    uint32_t code = zh->hash(key);  //获取key对应的hash值
    uint32_t entry_idx = code & (zh->nentries - 1); //位与运算=》hash值结合长度得到下标（解决问题1）
//见参考文件 本来entry_idx=code%length==>感觉就是将code放进长度为length的buckets组里 可以写成 entry_idx=code&(length-1)
//hashmap的存储机制实际上是数组加链表，在这里数组就代表桶的数量，检索时一次性就可以去掉n-1个区间
    while (zh->entries[entry_idx * zh->entrysz]) { //遍历hash表==》看看该位置是否为有效的，为0则跳出，1遍历，如果为1且当前key与设置的相同则进行覆盖，如果为0，表示即使hash同但是key不同，则跳出该循环，进行添加操作
        void *this_key = &zh->entries[entry_idx * zh->entrysz + 1];//在entity中获取key的地址
        void *this_value = &zh->entries[entry_idx * zh->entrysz + 1 + zh->keysz]; //在entity中获取value的地址

        if (zh->equals(key, this_key)) { //假如已经存在相关key
            // replace
            if (oldkey) //假如oldkey不为空
                memcpy(oldkey, this_key, zh->keysz);
            if (oldvalue)//假如oldvalue不为空
                memcpy(oldvalue, this_value, zh->valuesz);
            memcpy(this_key, key, zh->keysz);//用设置值对当前值进行覆盖，覆盖完了就返回
            memcpy(this_value, value, zh->valuesz);
            zh->entries[entry_idx * zh->entrysz] = 1; // mark valid==》从这里可以看出：hash的每个实体中，有一位表示是否有效 valid
            return 1; //如果存在，则返回旧的key和value，且函数结束返回1
        }//end of if
//hash映射就是把一堆东西分组，每组里面装着hash值相同的元素，每个东西的hash值只有一个，但一个hash值可对应若干东西
        entry_idx = (entry_idx + 1) & (zh->nentries - 1); //如果当前位置valid为1表示里面已经有内容了，则计算下一个地址。
    }//end of while

    // add the entry
    zh->entries[entry_idx * zh->entrysz] = 1; //将该空位置 标记为1， 表示里面有数据了（马上要存了）
    memcpy(&zh->entries[entry_idx * zh->entrysz + 1], key, zh->keysz);//赋值key
    memcpy(&zh->entries[entry_idx * zh->entrysz + 1 + zh->keysz], value, zh->valuesz);//赋值value
    zh->size++; //size+1

    if (zh->nentries < ZHASH_FACTOR_CRITICAL * zh->size) {//判读容量，不足则增加
        zhash_t *newhash = zhash_create_capacity(zh->keysz, zh->valuesz,
                                                 zh->hash, zh->equals,
                                                 zh->size);//用当前的zh->size来分配容量，因为之前的不满足要求，则生成足够多的空间（最终目的是空间换时间）

        for (int entry_idx = 0; entry_idx < zh->nentries; entry_idx++) {//不能超过分配的空间大小entry_idx < zh->nentries //遍历所有的

            if (zh->entries[entry_idx * zh->entrysz]) {//valid=1，表示存了东西，将其从久的hash表中复制到新的hash表中
                void *this_key = &zh->entries[entry_idx * zh->entrysz + 1]; //获取值key
                void *this_value = &zh->entries[entry_idx * zh->entrysz + 1 + zh->keysz];//取value
                if (zhash_put(newhash, this_key, this_value, NULL, NULL)) //放在新的hash表中
                    assert(0); // shouldn't already be present.
            }
        }//end of for

        // play switch-a-roo
        zhash_t tmp;
        memcpy(&tmp, zh, sizeof(zhash_t)); //zh对象是不变的呀，只是里面的内容变了（内容中的地址变了）
        memcpy(zh, newhash, sizeof(zhash_t)); //将newhash对象的内容复制过去（浅复制：并没有复制）
        memcpy(newhash, &tmp, sizeof(zhash_t));//将temp的内容复制给newhash
        zhash_destroy(newhash);//应该理解为管理指针才这么操作的吧？
    }//end of if

    return 0; //如果之前的key没有被存储，则添加后返回0
}

int zhash_remove(zhash_t *zh, const void *key, void *old_key, void *old_value)
{
    uint32_t code = zh->hash(key);
    uint32_t entry_idx = code & (zh->nentries - 1); //算index

    while (zh->entries[entry_idx * zh->entrysz]) {//valid=1，表示存在值
        void *this_key = &zh->entries[entry_idx * zh->entrysz + 1];
        void *this_value = &zh->entries[entry_idx * zh->entrysz + 1 + zh->keysz];

        if (zh->equals(key, this_key)) {//判断key，相等的情况
            if (old_key)
                memcpy(old_key, this_key, zh->keysz);
            if (old_value)
                memcpy(old_value, this_value, zh->valuesz);

            // mark this entry as available
            zh->entries[entry_idx * zh->entrysz] = 0;
            zh->size--;

            // reinsert any consecutive entries that follow
            while (1) {
                entry_idx = (entry_idx + 1) & (zh->nentries - 1); //判断下一个位置处是否有值（把下一个层的值往上一层提，把空的位置的下一层的值，移动过来）（因为同一个hash值可能有多个元素，第一个元素存在某个位置，第二个再存的时候，会判断valid值，0则可以存，1表示已经存了，则会生成新的位置，也会判断其valid，依次进行）

                if (zh->entries[entry_idx * zh->entrysz]) {//如果有
                    // completely remove this entry
                    char tmp[zh->entrysz];
                    memcpy(tmp, &zh->entries[entry_idx * zh->entrysz], zh->entrysz);
                    zh->entries[entry_idx * zh->entrysz] = 0;
                    zh->size--;
                    // reinsert it
                    if (zhash_put(zh, &tmp[1], &tmp[1+zh->keysz], NULL, NULL))
                        assert(0);
                } else {
                    break;
                }
            }
            return 1;
        }//end of if

        entry_idx = (entry_idx + 1) & (zh->nentries - 1);//不等则进行下一个位置的判断，知道valid=0
//（同一hash可能遇到几个不同的元素，即key不同）
    }

    return 0;
}

zhash_t *zhash_copy(const zhash_t *zh)
{
    zhash_t *newhash = zhash_create_capacity(zh->keysz, zh->valuesz,
                                             zh->hash, zh->equals,
                                             zh->size);

    for (int entry_idx = 0; entry_idx < zh->nentries; entry_idx++) {
        if (zh->entries[entry_idx * zh->entrysz]) {
            void *this_key = &zh->entries[entry_idx * zh->entrysz + 1];
            void *this_value = &zh->entries[entry_idx * zh->entrysz + 1 + zh->keysz];
            if (zhash_put(newhash, this_key, this_value, NULL, NULL)) //取久hash表的key/value对，再put到新的hash表中，最后返回新的hash表对象
                assert(0); // shouldn't already be present.
        }
    }

    return newhash;
}

int zhash_contains(const zhash_t *zh, const void *key)//判断是否包含key
{
    void *tmp;
    return zhash_get_volatile(zh, key, &tmp);
}

void zhash_iterator_init(zhash_t *zh, zhash_iterator_t *zit) //初始化一个iterator对象
{
    zit->zh = zh;
    zit->czh = zh;
    zit->last_entry = -1;
}

void zhash_iterator_init_const(const zhash_t *zh, zhash_iterator_t *zit)
{
    zit->zh = NULL;
    zit->czh = zh;
    zit->last_entry = -1;
}

int zhash_iterator_next_volatile(zhash_iterator_t *zit, void *outkey, void *outvalue)
{
    const zhash_t *zh = zit->czh;

    while (1) {
        if (zit->last_entry + 1 >= zh->nentries)
            return 0;

        zit->last_entry++;

        if (zh->entries[zit->last_entry * zh->entrysz]) {
            void *this_key = &zh->entries[zit->last_entry * zh->entrysz + 1];
            void *this_value = &zh->entries[zit->last_entry * zh->entrysz + 1 + zh->keysz];

            if (outkey != NULL)
                *((void**) outkey) = this_key;
            if (outvalue != NULL)
                *((void**) outvalue) = this_value;

            return 1;
        }
    }
}

int zhash_iterator_next(zhash_iterator_t *zit, void *outkey, void *outvalue)
{
    const zhash_t *zh = zit->czh;

    void *outkeyp, *outvaluep;

    if (!zhash_iterator_next_volatile(zit, &outkeyp, &outvaluep))
        return 0;

    if (outkey != NULL)
        memcpy(outkey, outkeyp, zh->keysz);
    if (outvalue != NULL)
        memcpy(outvalue, outvaluep, zh->valuesz);

    return 1;
}

void zhash_iterator_remove(zhash_iterator_t *zit)
{
    assert(zit->zh); // can't call _remove on a iterator with const zhash
    zhash_t *zh = zit->zh;

    zh->entries[zit->last_entry * zh->entrysz] = 0;
    zh->size--;

    // re-insert following entries
    int entry_idx = (zit->last_entry + 1) & (zh->nentries - 1);
    while (zh->entries[entry_idx *zh->entrysz]) {
        // completely remove this entry
        char tmp[zh->entrysz];
        memcpy(tmp, &zh->entries[entry_idx * zh->entrysz], zh->entrysz);
        zh->entries[entry_idx * zh->entrysz] = 0;
        zh->size--;

        // reinsert it
        if (zhash_put(zh, &tmp[1], &tmp[1+zh->keysz], NULL, NULL))
            assert(0);

        entry_idx = (entry_idx + 1) & (zh->nentries - 1);
    }

    zit->last_entry--;
}

void zhash_map_keys(zhash_t *zh, void (*f)())
{
    assert(zh != NULL);
    if (f == NULL)
        return;

    zhash_iterator_t itr;
    zhash_iterator_init(zh, &itr);

    void *key, *value;

    while(zhash_iterator_next_volatile(&itr, &key, &value)) {
        f(key);
    }
}

void zhash_vmap_keys(zhash_t * zh, void (*f)())
{
    assert(zh != NULL);
    if (f == NULL)
        return;

    zhash_iterator_t itr;
    zhash_iterator_init(zh, &itr);

    void *key, *value;

    while(zhash_iterator_next_volatile(&itr, &key, &value)) {
        void *p = *(void**) key;
        f(p);
    }
}

void zhash_map_values(zhash_t * zh, void (*f)())
{
    assert(zh != NULL);
    if (f == NULL)
        return;

    zhash_iterator_t itr;
    zhash_iterator_init(zh, &itr);

    void *key, *value;
    while(zhash_iterator_next_volatile(&itr, &key, &value)) {
        f(value);
    }
}

void zhash_vmap_values(zhash_t * zh, void (*f)())
{
    assert(zh != NULL);
    if (f == NULL)
        return;

    zhash_iterator_t itr;
    zhash_iterator_init(zh, &itr);

    void *key, *value;
    while(zhash_iterator_next_volatile(&itr, &key, &value)) {
        void *p = *(void**) value;
        f(p);
    }
}

zarray_t *zhash_keys(const zhash_t *zh)//所有key放在一个array中
{
    assert(zh != NULL);

    zarray_t *za = zarray_create(zh->keysz);

    zhash_iterator_t itr;
    zhash_iterator_init_const(zh, &itr);

    void *key, *value;
    while(zhash_iterator_next_volatile(&itr, &key, &value)) {
        zarray_add(za, key);
    }

    return za;
}

zarray_t *zhash_values(const zhash_t *zh) //将所有值value放在一个array中
{
    assert(zh != NULL);

    zarray_t *za = zarray_create(zh->valuesz);

    zhash_iterator_t itr;
    zhash_iterator_init_const(zh, &itr);

    void *key, *value;
    while(zhash_iterator_next_volatile(&itr, &key, &value)) {
        zarray_add(za, value);
    }

    return za;
}


uint32_t zhash_uint32_hash(const void *_a)
{
    assert(_a != NULL);

    uint32_t a = *((uint32_t*) _a);
    return a;
}

int zhash_uint32_equals(const void *_a, const void *_b)
{
    assert(_a != NULL);
    assert(_b != NULL);

    uint32_t a = *((uint32_t*) _a);
    uint32_t b = *((uint32_t*) _b);

    return a==b;
}

uint32_t zhash_uint64_hash(const void *_a)
{
    assert(_a != NULL);

    uint64_t a = *((uint64_t*) _a);
    return (uint32_t) (a ^ (a >> 32));
}

int zhash_uint64_equals(const void *_a, const void *_b)
{
    assert(_a != NULL);
    assert(_b != NULL);

    uint64_t a = *((uint64_t*) _a);
    uint64_t b = *((uint64_t*) _b);

    return a==b;
}


union uintpointer
{
    const void *p;
    uint32_t i;
};

uint32_t zhash_ptr_hash(const void *a)
{
    assert(a != NULL);

    union uintpointer ip;
    ip.p = * (void**)a;

    // compute a hash from the lower 32 bits of the pointer (on LE systems)
    uint32_t hash = ip.i;
    hash ^= (hash >> 7);

    return hash;
}


int zhash_ptr_equals(const void *a, const void *b)
{
    assert(a != NULL);
    assert(b != NULL);

    const void * ptra = * (void**)a;
    const void * ptrb = * (void**)b;
    return  ptra == ptrb;
}


int zhash_str_equals(const void *_a, const void *_b) //判断两个字符串是否相等
{
    assert(_a != NULL);
    assert(_b != NULL);

    char *a = * (char**)_a; //void* 指针在使用前需要转换成具体的指针类型
    char *b = * (char**)_b;

    return !strcmp(a, b);
}

uint32_t zhash_str_hash(const void *_a)//计算string的hash值
{
    assert(_a != NULL);//判断指针是否为真

    char *a = * (char**)_a;//转成char型指针

    int32_t hash = 0;
    while (*a != 0) { //不为空
        hash = (hash << 7) + (hash >> 23);
        hash += *a;
        a++;
    }

    return (uint32_t) hash; //返回hash散列值
}


void zhash_debug(zhash_t *zh)
{
    for (int entry_idx = 0; entry_idx < zh->nentries; entry_idx++) {
        char *k, *v;
        memcpy(&k, &zh->entries[entry_idx * zh->entrysz + 1], sizeof(char*));
        memcpy(&v, &zh->entries[entry_idx * zh->entrysz + 1 + zh->keysz], sizeof(char*));
        printf("%d: %d, %s => %s\n", entry_idx, zh->entries[entry_idx * zh->entrysz], k, v);
    }
}
