#define LIST_SIZE 20

typedef struct List {
    uint8_t items[LIST_SIZE];
    uint8_t amount;
} List;

void init_list(List *list) {
    list->amount = 0;
    for (uint8_t i = 0; i < LIST_SIZE; ++i) {
        list->items[i] = 0;
    }
}

void add_to_list(List *list, uint8_t item) {
    if(item==0){
        return;
    }
    for (uint8_t i = 0; i < list->amount; ++i) {
        if (list->items[i] == item) {
            return;
        }
    }
    list->items[list->amount] = item;
    list->amount++;
}

void remove_from_list(List *list, uint8_t item) {
    for (uint8_t i = 0; i < list->amount; ++i) {
        if (list->items[i] == item) {
            list->items[i] = 0;
            for (uint8_t j = i; j < list->amount; j++) {
                if (list->items[j + 1] != 0) {
                    list->items[j] = list->items[j + 1];
                }
            }
            list->amount--;
            return;
        }
    }
}

uint8_t pop_first_from_list(List *list){
    if(list->amount==0){
        return 0;
    }
    uint8_t tmp=list->items[0];
    remove_from_list(list, tmp);
    return tmp;
}

uint8_t is_in_list(List list, uint8_t item){
    for (uint8_t i = 0; i < list.amount; ++i) {
        if (list.items[i] == item) {
            return 1;
        }
    }
    return 0;
}