#include "peripheral_alloc.h"

extern "C" {

void *wifi_alloc_peripheral(void *device)
{
    return (void *)codal::allocate_peripheral(device);
}

void wifi_set_irq(void *device, void (*fn)(void *), void *userdata)
{
    codal::set_alloc_peri_irq(device, fn, userdata);
}

}
