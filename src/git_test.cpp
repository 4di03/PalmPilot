

int testFunction(unsigned char a, unsigned char b)
{
    unsigned char product = 0;
    for (unsigned char i = 0; i < a; i++)
    {
        for (unsigned char j = 0; j < b; j++)
        {

            product += 1;
        }
    }

    return product;
}


int smartProduct(int a, int b)
{
    return a * b;
}

