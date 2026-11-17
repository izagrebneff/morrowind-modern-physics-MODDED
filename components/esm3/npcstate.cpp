#include "npcstate.hpp"
#include "esmreader.hpp"
#include "esmwriter.hpp"

namespace ESM
{

    void NpcState::load(ESMReader& esm)
    {
        ObjectState::load(esm);
        mBloodDecals.clear();

        if (mHasCustomState)
        {
            mInventory.load(esm);

            mNpcStats.load(esm);

            mCreatureStats.load(esm);
            const auto encoded = esm.getHNOString("NBLO");
            if (encoded.size() <= 800000 && encoded.size() % 2 == 0)
            {
                auto nibble = [](char c) -> int {
                    if (c >= '0' && c <= '9') return c - '0';
                    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                    return -1;
                };
                mBloodDecals.reserve(encoded.size() / 2);
                for (std::size_t i = 0; i < encoded.size(); i += 2)
                {
                    const int hi = nibble(encoded[i]), lo = nibble(encoded[i + 1]);
                    if (hi < 0 || lo < 0) { mBloodDecals.clear(); break; }
                    mBloodDecals.push_back(static_cast<char>((hi << 4) | lo));
                }
            }
        }
    }

    void NpcState::save(ESMWriter& esm, bool inInventory) const
    {
        ObjectState::save(esm, inInventory);

        if (mHasCustomState)
        {
            mInventory.save(esm);

            mNpcStats.save(esm);

            mCreatureStats.save(esm);
            if (!mBloodDecals.empty() && mBloodDecals.size() <= 400000)
            {
                constexpr char digits[] = "0123456789abcdef";
                std::string encoded;
                encoded.reserve(mBloodDecals.size() * 2);
                for (unsigned char c : mBloodDecals)
                {
                    encoded.push_back(digits[c >> 4]);
                    encoded.push_back(digits[c & 15]);
                }
                esm.writeHNString("NBLO", encoded);
            }
        }
    }

    void NpcState::blank()
    {
        ObjectState::blank();
        mNpcStats.blank();
        mCreatureStats.blank();
        mBloodDecals.clear();
        mHasCustomState = true;
    }

}
