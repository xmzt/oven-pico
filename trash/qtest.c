#include <stdio.h>

typedef unsigned int uint;


//-----------------------------------------------------------------------------------------------------------------------
// zqi item

typedef struct zqi_t zqi_t;

typedef void zqi_fun_t(zqi_t *self);

struct zqi_t {
	zqi_t *ab;
	zqi_fun_t *fun;
};

inline static void zqi_init(zqi_t *self, zqi_fun_t *fun) {
	self->ab = (void*)0;
	self->fun = fun;
}

int zqi_id(zqi_t *i); 
void zq_dump();

//-----------------------------------------------------------------------------------------------------------------------
// zq list
//
// item->ab == item signifies item is tail of list
// item->ab == 0 signifies item not in list
// this can be used to avoid calling za_enq on an item already in list.

typedef struct {
	zqi_t *a;
	zqi_t **enq_dst;
} zq_t;

void zq_init(zq_t *self) {
	self->a = NULL;
	self->enq_dst = &self->a;
}

void zq_enq(zq_t *self, zqi_t *item) {
	printf("[zq_enq] %d\n", zqi_id(item));
	item->ab = item;
	// sync {
	*self->enq_dst = item;
	self->enq_dst = &item->ab;
	// sync }
	zq_dump();
}

void zq_con1(zq_t *self) {
	// don't really need to include self->a in sync.
	// enq can only change self->a from 0 to non-null, which happens after self->a->ab is initialized.
	// same as an atomic operation
	zqi_t *a = self->a;
	if(a) {
		// sync {
		self->a = a->ab;
		if(self->a == a)
			zq_init(self); // at tail of list. reset overall list for subsequent enq.
		// sync }
		printf("[zq_con1] %d\n", zqi_id(a));
		a->fun(a);
		zq_dump();
	} else printf("[zq_con1] -\n");
}

//-----------------------------------------------------------------------------------------------------------------------
// zq test

zqi_t test_zqiV[3];
zq_t test_zq;

int zqi_id(zqi_t *i) {
	return ! i ? 0
		: (i - test_zqiV) + 1;
}

void zq_dump() {
	printf("    [test_zq] a=%d\n", zqi_id(test_zq.a));
	for(zqi_t *i = test_zq.a; i; i = i->ab) {
		printf("        zqi=%d .ab=%d\n", zqi_id(i), zqi_id(i->ab));
		if(i->ab == i) break;
	}
	printf("    [test_zqiV]\n");
	for(zqi_t *i = test_zqiV, *j = i + 3; i < j; i++)
		printf("        zqiV %d .ab=%d\n", zqi_id(i), zqi_id(i->ab));
}

void test_zqi_fun(zqi_t *self) {
	printf("[test_zqi_fun] %d\n", zqi_id(self));
	self->ab = NULL;
}

int main(int argc, char **argv) {
	zq_init(&test_zq);
	for(uint i = 0; i < 3; i++)
		zqi_init(test_zqiV + i, test_zqi_fun);
	zq_dump();

	zq_enq(&test_zq, test_zqiV + 2);
	zq_con1(&test_zq);
	zq_con1(&test_zq);
	zq_enq(&test_zq, test_zqiV + 1);
	zq_enq(&test_zq, test_zqiV + 0);
	zq_con1(&test_zq);
	zq_enq(&test_zq, test_zqiV + 2);
	zq_con1(&test_zq);
	zq_con1(&test_zq);
	zq_con1(&test_zq);
	zq_con1(&test_zq);
}
