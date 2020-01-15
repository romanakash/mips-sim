/*************************************************************************************|
|   1. YOU ARE NOT ALLOWED TO SHARE/PUBLISH YOUR CODE (e.g., post on piazza or online)|
|   2. Fill main.c and memory_hierarchy.c files                                       |
|   3. Do not use any other .c files neither alter main.h or parser.h                 |
|   4. Do not include any other library files                                         |
|*************************************************************************************/
#include "mipssim.h"

/// @students: declare cache-related structures and variables here
typedef struct cache_block
{
    int valid_bit;
    int tag;
    int *data; // contains array of 4 ints to represent 4 words
} cache_block_t;

cache_block_t *cache_array;

int get_data_index(int byte_offset)
{
    // address is always a multiple of 4
    // each block starts from a mulitple of 16
    if (byte_offset == 0)
        return 0;
    else if (byte_offset == 4)
        return 1;
    else if (byte_offset == 8)
        return 2;
    else if (byte_offset == 12)
        return 3;
    else
    {
        printf("INVALID BYTE OFFSET: %d\n", byte_offset);
        assert(false);
    }
}

void memory_state_init(struct architectural_state *arch_state_ptr)
{
    arch_state_ptr->memory = (uint32_t *)malloc(sizeof(uint32_t) * MEMORY_WORD_NUM);
    memset(arch_state_ptr->memory, 0, sizeof(uint32_t) * MEMORY_WORD_NUM);
    if (cache_size == 0)
    {
        // CACHE DISABLED
        memory_stats_init(arch_state_ptr, 0); // WARNING: we initialize for no cache 0
    }
    else
    {
        // CACHE ENABLED
        //assert(0); /// @students: Remove assert(0); and initialize cache

        /// @students: memory_stats_init(arch_state_ptr, X); <-- fill # of tag bits for cache 'X' correctly
        int num_of_words = cache_size / 4;
        int num_of_blocks = num_of_words / 4;

        int index_bits = 0;

        while (num_of_blocks >= 2)
        {
            num_of_blocks /= 2;
            index_bits++;
        }

        // calculate again after dividing
        num_of_blocks = num_of_words / 4;

        // 32 - (n + m +2)
        int X = 32 - (index_bits + 2 + 2);
        printf("num_of_blocks, n, X %d, %d, %d\n", num_of_blocks, index_bits, X);

        memory_stats_init(arch_state_ptr, X);

        // cache allocation
        cache_array = calloc(num_of_blocks, sizeof(cache_block_t));

        // init every struct and allocate space for data array
        for (int i = 0; i < num_of_blocks; i++)
        {
            // You don't need this line but don't remove it
            cache_block_t block = {.valid_bit = 0};
            cache_array[i].data = calloc(4, sizeof(int));
        }
    }
}

// returns data on memory[address / 4]
int memory_read(int address)
{
    arch_state.mem_stats.lw_total++;
    check_address_is_word_aligned(address);

    if (cache_size == 0)
    {
        // CACHE DISABLED
        return (int)arch_state.memory[address / 4];
    }
    else
    {
        // CACHE ENABLED
        //assert(0); /// @students: Remove assert(0); and implement Memory hierarchy w/ cache

        /// @students: your implementation must properly increment: arch_state_ptr->mem_stats.lw_cache_hits
        int num_of_words = cache_size / 4;
        int num_of_blocks = num_of_words / 4;

        int index_bits = 0;

        while (num_of_blocks >= 2)
        {
            num_of_blocks /= 2;
            index_bits++;
        }

        // calculate again after dividing
        num_of_blocks = num_of_words / 4;

        int byte_offset = get_piece_of_a_word(address, 0, 4);
        int index = get_piece_of_a_word(address, 4, index_bits);
        int tag = get_piece_of_a_word(address, 4 + index_bits, arch_state.bits_for_cache_tag);

        int data_index = get_data_index(byte_offset);

        cache_block_t *block = &cache_array[index % num_of_blocks];

        // hit scenario
        printf("vB %d\n", block->valid_bit);
        if (block->valid_bit)
        {
            if (tag == block->tag)
            {
                arch_state.mem_stats.lw_cache_hits++;

                printf("LW index: %d \n", index % num_of_blocks);
                printf("LW tag: %d %d\n", tag, block->tag);
                printf("LW byte offset: %d\n", byte_offset);
                printf("LW CACHE HIT: %d\n", (int)block->data[data_index]);
                printf("LW HITS: %d\n", (int)arch_state.mem_stats.lw_cache_hits);

                return (int)block->data[data_index];
            }
        }

        // miss scenario
        block->valid_bit = 1;
        block->tag = tag;

        int block_start_address = address - (data_index * 4);

        // fetch the whole block from memory
        block->data[0] = (int)arch_state.memory[(block_start_address + 0) / 4];
        block->data[1] = (int)arch_state.memory[(block_start_address + 4) / 4];
        block->data[2] = (int)arch_state.memory[(block_start_address + 8) / 4];
        block->data[3] = (int)arch_state.memory[(block_start_address + 12) / 4];

        return (int)arch_state.memory[address / 4];
    }
    return 0;
}

// writes data on memory[address / 4]
void memory_write(int address, int write_data)
{
    arch_state.mem_stats.sw_total++;
    check_address_is_word_aligned(address);

    if (cache_size == 0)
    {
        // CACHE DISABLED
        arch_state.memory[address / 4] = (uint32_t)write_data;
    }
    else
    {
        // CACHE ENABLED
        //assert(0); /// @students: Remove assert(0); and implement Memory hierarchy w/ cache

        /// @students: your implementation must properly increment: arch_state_ptr->mem_stats.sw_cache_hits
        int num_of_words = cache_size / 4;
        int num_of_blocks = num_of_words / 4;

        int index_bits = 0;

        while (num_of_blocks >= 2)
        {
            num_of_blocks /= 2;
            index_bits++;
        }

        // calculate again after dividing
        num_of_blocks = num_of_words / 4;

        int byte_offset = get_piece_of_a_word(address, 0, 4);
        int index = get_piece_of_a_word(address, 4, index_bits);
        int tag = get_piece_of_a_word(address, 4 + index_bits, arch_state.bits_for_cache_tag);

        int data_index = get_data_index(byte_offset);

        cache_block_t *block = &cache_array[index % num_of_blocks];

        // hit scenario
        if (block->valid_bit)
        {
            if (tag == block->tag)
            {
                arch_state.mem_stats.sw_cache_hits++;

                printf("SW index: %d \n", index % num_of_blocks);
                printf("SW tag: %d %d\n", tag, block->tag);
                printf("SW byte offset: %d\n", byte_offset);
                printf("SW CACHE HIT: %d\n", (int)block->data[data_index]);
                printf("SW HITS: %d\n", (int)arch_state.mem_stats.sw_cache_hits);

                // write to cache
                block->data[data_index] = (uint32_t)write_data;
                // write to memory
                arch_state.memory[address / 4] = (uint32_t)write_data;

                return;
            }
        }

        // miss scenario
        // write to memory
        arch_state.memory[address / 4] = (uint32_t)write_data;
    }
}
