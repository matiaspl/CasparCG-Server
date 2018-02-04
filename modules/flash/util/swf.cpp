/*
 * Copyright (c) 2011 Sveriges Television AB <info@casparcg.com>
 *
 * This file is part of CasparCG (www.casparcg.com).
 *
 * CasparCG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CasparCG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CasparCG. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Robert Nagy, ronag89@gmail.com
 */

#include "../StdAfx.h"

#include "swf.h"

#include <common/except.h>

#include <zlib.h> // Compiled into FreeImage

#include <fstream>
#include <streambuf>

namespace caspar { namespace flash {

std::vector<char> decompress_one_file(const std::vector<char>& in_data, uLong buf_size = 5000000)
{
    if (buf_size > 300 * 1000000)
        CASPAR_THROW_EXCEPTION(file_read_error());

    std::vector<char> out_data(buf_size, 0);

    auto ret = uncompress(reinterpret_cast<Bytef*>(out_data.data()),
                          &buf_size,
                          reinterpret_cast<const Bytef*>(in_data.data()),
                          static_cast<uLong>(in_data.size()));

    if (ret == Z_BUF_ERROR)
        return decompress_one_file(in_data, buf_size * 2);

    if (ret != Z_OK)
        CASPAR_THROW_EXCEPTION(file_read_error());

    out_data.resize(buf_size);

    return out_data;
}

std::string read_template_meta_info(const std::wstring& filename)
{
    auto file = std::fstream(filename, std::ios::in | std::ios::binary);

    if (!file)
        CASPAR_THROW_EXCEPTION(file_read_error());

    char head[4] = {};
    file.read(head, 3);

    std::vector<char> data;

    file.seekg(0, std::ios::end);
    data.reserve(static_cast<size_t>(file.tellg()));
    file.seekg(0, std::ios::beg);

    if (strcmp(head, "CWS") == 0) {
        file.seekg(8, std::ios::beg);
        std::copy((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>(), std::back_inserter(data));
        data = decompress_one_file(data);
    } else {
        file.seekg(0, std::ios::end);
        data.reserve(static_cast<size_t>(file.tellg()));
        file.seekg(0, std::ios::beg);

        std::copy((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>(), std::back_inserter(data));
    }

    std::string beg_str = "<template version";
    std::string end_str = "</template>";
    auto        beg_it  = std::find_end(data.begin(), data.end(), beg_str.begin(), beg_str.end());
    auto        end_it  = std::find_end(beg_it, data.end(), end_str.begin(), end_str.end());

    if (beg_it == data.end() || end_it == data.end())
        CASPAR_THROW_EXCEPTION(file_read_error());

    return std::string(beg_it, end_it + end_str.size());
}

swf_t::header_t::header_t(const std::wstring& filename)
    : valid(false)
{
    auto stream = std::fstream();
    stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    stream.open(filename, std::ios::in | std::ios::binary);
    stream.read(reinterpret_cast<char*>(this), 8);

    const std::array<std::uint8_t, 3> s1 = {'F', 'W', 'S'};
    const std::array<std::uint8_t, 3> s2 = {'C', 'W', 'S'};

    if (this->signature != s1 && this->signature != s2)
        return;

    _byteswap_ulong(this->file_length);

    std::vector<char> file_data;
    std::copy(
        (std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>(), std::back_inserter(file_data));

    std::array<char, 32> uncompressed_data;
    uLongf               file_size = 32;
    auto                 ret       = uncompress(reinterpret_cast<Bytef*>(uncompressed_data.data()),
                          &file_size,
                          reinterpret_cast<const Bytef*>(file_data.data()),
                          static_cast<uLong>(file_data.size()));

    if (ret == Z_DATA_ERROR)
        CASPAR_THROW_EXCEPTION(io_error());

    // http://thenobody.blog.matfyz.sk/p13084-how-to-get-dimensions-of-a-swf-file

    unsigned char nbits = reinterpret_cast<unsigned char*>(uncompressed_data.data())[0];

    unsigned int size = nbits >> 3; // remove overlaping 3 bits

    unsigned long dims[4]  = {};
    unsigned long neg_root = 1 << (size - 1); // numbers are signed, i.e. leftmost bit denotes -

    unsigned int  bi_offset = (size % 8) ? (8 - (size % 8)) : 0; // offset of bit numbers depending on specified size
    unsigned int  by_offset = (size + bi_offset) / 8;            // offest of bytes
    unsigned int  ioffset;                                       // floating byte offset during iteration
    unsigned long ibuf = (unsigned long)(nbits % 8); // actual result - starts with last 3 bits of first byte

    for (auto i = 0; i < 4; ++i) {
        ioffset = by_offset * i;

        for (unsigned int j = 0; j < by_offset; ++j) {
            ibuf <<= 8;
            ibuf += reinterpret_cast<unsigned char*>(uncompressed_data.data())[1 + ioffset + j];
        }

        dims[i] = (ibuf >> (3 + bi_offset + (i * bi_offset))) / 20; // coordinates in twips, so divide by 20 for pixels

        if (dims[i] >= neg_root)                                // if the leftmost bit is 1 number is negative
            dims[i] = (-1) * (neg_root - (dims[i] - neg_root)); // convert to negative number

        int expn = 3 + bi_offset + (i * bi_offset); // bit divider for ...
        ibuf     = ibuf % (1 << (expn - 1));        // ... buffered number
    }

    this->frame_width  = dims[1] - dims[0]; // max - mix
    this->frame_height = dims[3] - dims[2];

    this->valid = true;
}

swf_t::swf_t(const std::wstring& filename)
    : header(filename)
{
    auto stream = std::fstream();
    stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    stream.open(filename, std::ios::in | std::ios::binary);
    stream.seekg(8, std::fstream::beg);

    this->data.resize(this->header.file_length - 8);

    std::vector<char> file_data;
    std::copy(
        (std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>(), std::back_inserter(file_data));

    uLongf file_size = this->header.file_length;
    auto   ret       = uncompress(reinterpret_cast<Bytef*>(this->data.data()),
                          &file_size,
                          reinterpret_cast<const Bytef*>(file_data.data()),
                          static_cast<uLong>(file_data.size()));

    if (ret != Z_OK)
        CASPAR_THROW_EXCEPTION(io_error());
}

}} // namespace caspar::flash
