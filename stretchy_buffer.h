#pragma once

template <typename T, bool ABC = RAY_USE_ABC>
struct StretchyBuffer {
    Arena* arena;

    u32 capacity;
    u32 count;
    T*  data;

    //

    static StretchyBuffer
    on_arena(Arena* arena) {
        StretchyBuffer result = {};
        result.arena = arena;
        return result;
    }

    force_inline T&
    operator[](usize index) const {
        if constexpr(ABC) {
			assert(index < count);
        }
        return data[index];
    }

    void
    resize(u32 new_capacity) {
        T* new_data = 0;

        u32 amount_to_grow = (new_capacity - capacity);

        if (arena) {
            usize arena_size_remaining = arena->get_size_remaining(alignof(T));
            u32 amount_that_can_fit = (u32)(arena_size_remaining / sizeof(T));

            assert(amount_that_can_fit > 0);

            if (amount_to_grow > amount_that_can_fit) {
                amount_to_grow = amount_that_can_fit;
            }

            T* next_location = data + count;
            T* next_allocation_location = (T*)arena->get_next_allocation_location(alignof(T));
            if (data && (next_location == next_allocation_location)) {
                // NOTE: We can grow in place
                push_array(arena, amount_to_grow, T, clear(alignof(T)));
                new_data = data;
            } else {
                new_data = push_array(arena, new_capacity, T, clear(alignof(T)));
				if (data) {
                    memcpy(new_data, data, sizeof(T)*count);
                }
            }
        } else {
            new_data = (T*)realloc(data, sizeof(T)*new_capacity);
            if (!new_data) {
                ::free(new_data);
                INVALID_CODE_PATH;
            }
            // MOTE: The policy of this stretchy buffer is to use zeroed memory
            if (data) {
				memset(new_data + capacity, 0, sizeof(T)*amount_to_grow);
            } else {
                memset(new_data, 0, sizeof(T)*new_capacity);
            }
        }

        assert(new_data);

#pragma warning(suppress: 6001) // It's fine that it's uninitialized data, my overly sensitive intellisense.
        data = new_data;
        capacity = new_capacity;
    }

    void
    trim_arena() {
        if (arena) {
            char* arena_at        = arena->get_next_allocation_location(alignof(T));
            char* capacity_end_at = (char*)(data + capacity);
            if (arena_at == capacity_end_at) {
                char* data_end_at = (char*)(data + count);
                arena->reset_to(data_end_at);
            }
        } else {
            INVALID_CODE_PATH;
        }
    }

    void
    ensure_space(u32 required_count) {
        if ((count + required_count) > capacity) {
            u32 new_capacity = Max(8, Max(count + required_count, 2*capacity));
            resize(new_capacity);
        }
    }

    T*
    add(const T& value) {
        ensure_space(1);

        T* result = &data[count++];
        assert_slow(memory_is_zeroed(sizeof(T), (char*)result));

        *result = value;

        return result;
    }

    T*
    add() {
        ensure_space(1);

        T* result = &data[count++];
        assert_slow(memory_is_zeroed(sizeof(T), (char*)result));

        return result;
    }

    T*
    add_n(u32 n) {
        ensure_space(n);

        T* result = &data[count];
        count += n;

        assert_slow(memory_is_zeroed(sizeof(T)*n, (char*)result));

        return result;
    }

    void
    append(u32 append_count, T* append_data) {
        ensure_space(append_count);
        memcpy(data + count, append_data, sizeof(*append_data)*append_count);
        count += append_count;
    }

    void
    clear_data_to_zero() const {
        memset(data, 0, sizeof(T)*capacity);
    }

    void
    free() {
        if (arena) {
            INVALID_CODE_PATH;
        } else if (data) {
            ::free(data);
        }
        zero_struct(this);
    }

private:
    static b32
    memory_is_zeroed(u32 byte_count, char* bytes) {
        b32 result = true;
        for (usize i = 0; i < byte_count; ++i) {
            if (bytes[i] != 0) {
                result = false;
                break;
            }
        }
        return result;
    }
};

template <typename T, bool ABC = RAY_USE_ABC>
struct Array {
    usize count;
    T* data;

    //

    force_inline T&
    operator[](usize index) const {
        if constexpr(ABC) {
			assert(index < count);
        }
        return data[index];
    }

    static Array
    from(const StretchyBuffer<T>& buffer) {
        Array result;
        result.count = buffer.count;
        result.data  = buffer.data;
        return result;
    }

    static Array
    from(usize count, T* data) {
        Array result;
        result.count = count;
        result.data  = data;
        return result;
    }
};

#pragma warning(disable: 6255) // NOTE: This warning says to use malloca instead. But malloca
                               //       is the dumbest thing ever, it just plainly falls back
                               //       on malloc if you allocate more than _ALLOCA_S_THRESHOLD
                               //       (which is usually set to 1024 bytes), which is utterly
                               //       useless. Literally have no idea what on god's green earth
							   //       that is supposed to accomplish except cause memory leaks.
                               //       Good lord.
#define array_on_stack(T, capacity) Array<T>::from(capacity, (T*)alloca(sizeof(T)*capacity))
