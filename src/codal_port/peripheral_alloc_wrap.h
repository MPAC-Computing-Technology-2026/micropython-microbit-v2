#ifndef PERIPHERAL_ALLOC_WRAP_H
#define PERIPHERAL_ALLOC_WRAP_H

#ifdef __cplusplus
extern "C" {
#endif

void *wifi_alloc_peripheral(void *device);
void wifi_set_irq(void *device, void (*fn)(void *), void *userdata);

#ifdef __cplusplus
}
#endif

#endif
