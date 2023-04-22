#ifndef MAINQ_H
#define MAINQ_H

//-----------------------------------------------------------------------------------------------------------------------
// mainq
//
// a: start of list
// b: tail of list
// ab: link towards b
//
// item->ab == item signifies item is tail of list
// item->ab == 0 signifies item not in list
// this can be used to avoid calling za_enq on an item already in list.

typedef struct mainq_item_t mainq_item_t;

typedef void mainq_item_fun_t(volatile mainq_item_t *self);

struct mainq_item_t {
	volatile mainq_item_t *ab;
	mainq_item_fun_t *fun;
};

typedef struct {
	volatile mainq_item_t *a;
	volatile mainq_item_t **enq_dst;
} mainq_t;

inline static void mainq_item_init(mainq_item_t *self, mainq_item_fun_t *fun) {
	self->ab = (void*)0;
	self->fun = fun;
}

inline static void mainq_init(mainq_t *self) {
	self->a = (void*)0;
	self->enq_dst = &self->a;
}

inline static void mainq_enq(mainq_t *self, mainq_item_t *item) {
	item->ab = item;
	// sync {
	*self->enq_dst = item;
	self->enq_dst = &item->ab;
	// sync }
}

// this function here as a template. variations expected in actual implementation.
inline static void mainq_con1(mainq_t *self) {
	// should not need to include self->a in sync:
	// enq can only change self->a from 0 to non-null, which happens after self->a->ab is initialized.
	// same as an atomic operation
	volatile mainq_item_t *a = self->a;
	if(a) {
		// sync {
		self->a = a->ab;
		if(self->a == a)
			mainq_init(self); // at tail of list. reset overall list for subsequent enq.
		// sync }
		a->fun(a);
	}
}

extern mainq_t g_mainq;

#endif
