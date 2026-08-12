#ifndef OSRM_CONTRACTOR_ORDINAL_IDENTITY_HPP
#define OSRM_CONTRACTOR_ORDINAL_IDENTITY_HPP

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace osrm::contractor::phast
{

struct OrdinalsIdentity
{
    std::uint64_t sample_count = 0;
    std::uint64_t ordinals_file_size = 0;
    std::uint64_t ordinals_hash_hi = 0;
    std::uint64_t ordinals_hash_lo = 0;

    bool operator==(const OrdinalsIdentity &other) const
    {
        return sample_count == other.sample_count &&
               ordinals_file_size == other.ordinals_file_size &&
               ordinals_hash_hi == other.ordinals_hash_hi &&
               ordinals_hash_lo == other.ordinals_hash_lo;
    }
};

struct OrdinalsIdentityFile
{
    std::array<char, 8> magic = {'P', 'H', 'O', 'R', 'D', 'I', '1', '\0'};
    std::uint32_t version = 1;
    std::uint32_t resolution = 0;
    OrdinalsIdentity identity;
};
static_assert(sizeof(OrdinalsIdentityFile) == 48);

inline std::filesystem::path OrdinalsIdentityPath(const std::filesystem::path &ordinals_path)
{
    return std::filesystem::path{ordinals_path.string() + ".identity"};
}

inline bool ReadOrdinalsIdentity(const std::filesystem::path &ordinals_path,
                                 const std::uint32_t expected_resolution,
                                 OrdinalsIdentity &identity,
                                 std::string &error)
{
    const auto path = OrdinalsIdentityPath(ordinals_path);
    OrdinalsIdentityFile file;
    std::ifstream input(path, std::ios::binary);
    input.read(reinterpret_cast<char *>(&file), sizeof(file));
    const std::array<char, 8> expected_magic = {'P', 'H', 'O', 'R', 'D', 'I', '1', '\0'};
    if (!input || input.peek() != std::char_traits<char>::eof() || file.magic != expected_magic ||
        file.version != 1 || file.resolution != expected_resolution ||
        file.identity.sample_count == 0 ||
        file.identity.sample_count > std::numeric_limits<std::uint64_t>::max() / 8 ||
        file.identity.ordinals_file_size != file.identity.sample_count * sizeof(std::uint64_t))
    {
        error = "Invalid ordinal identity: " + path.string();
        return false;
    }
    std::error_code size_error;
    if (std::filesystem::file_size(ordinals_path, size_error) !=
            file.identity.ordinals_file_size ||
        size_error)
    {
        error = "Ordinal file size does not match identity: " + ordinals_path.string();
        return false;
    }
    identity = file.identity;
    return true;
}

inline bool WriteOrdinalsIdentity(const std::filesystem::path &ordinals_path,
                                  const std::uint32_t resolution,
                                  const OrdinalsIdentity &identity,
                                  std::string &error)
{
    const auto path = OrdinalsIdentityPath(ordinals_path);
    const auto temporary = std::filesystem::path{path.string() + ".tmp"};
    const OrdinalsIdentityFile file{{'P', 'H', 'O', 'R', 'D', 'I', '1', '\0'},
                                    1,
                                    resolution,
                                    identity};
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    output.write(reinterpret_cast<const char *>(&file), sizeof(file));
    output.close();
    if (!output)
    {
        error = "Failed writing ordinal identity: " + temporary.string();
        return false;
    }
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    std::error_code rename_error;
    std::filesystem::rename(temporary, path, rename_error);
    if (rename_error)
    {
        error = "Failed finalizing ordinal identity: " + rename_error.message();
        std::filesystem::remove(temporary, ignored);
        return false;
    }
    return true;
}

} // namespace osrm::contractor::phast

#endif
