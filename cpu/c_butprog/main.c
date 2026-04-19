#define disp ((volatile uint16_t*)0x20)
#define but ((volatile uint8_t*)0x0)

int main(void)
{
    while (1) {
        *disp = *but;
    }
}