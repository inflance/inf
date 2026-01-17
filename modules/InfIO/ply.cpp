/// @file ply.cpp
/// @brief PLY 文件格式读写实现

#include "ply.hpp"
#include "macro.hpp"

#include <charconv>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <cmath>

namespace inf::io
{
	namespace
	{
		[[nodiscard]] const char* skip_whitespace(const char* p, const char* end)
		{
			while (p < end && (*p == ' ' || *p == '\t')) ++p;
			return p;
		}

		template <typename T>
		[[nodiscard]] const char* parse_int(const char* p, const char* end, T& value)
		{
			p = skip_whitespace(p, end);
			auto result = std::from_chars(p, end, value);
			return (result.ec == std::errc{}) ? result.ptr : nullptr;
		}

		[[nodiscard]] const char* parse_double(const char* p, const char* end, double& value)
		{
			p = skip_whitespace(p, end);
			char* endptr = nullptr;
			value = std::strtod(p, &endptr);
			return (endptr > p) ? endptr : nullptr;
		}

		[[nodiscard]] uint8_t clamp_color(float value)
		{
			return static_cast<uint8_t>(std::clamp(value, 0.0f, 255.0f));
		}

		[[nodiscard]] core::Error file_error(const std::filesystem::path& path, const std::string& msg)
		{
			return {core::ErrorCode::kFileNotFound, msg + ": " + path.string()};
		}

		[[nodiscard]] core::Error parse_error(const std::string& msg)
		{
			return {core::ErrorCode::kParseError, msg};
		}

		// ============================================================================
		// Header 写入
		// ============================================================================

		void write_mesh_header(std::ostream& os, size_t num_vertices, size_t num_faces,
		                       bool has_normals, bool binary)
		{
			os << "ply\n";
			os << "format " << (binary ? "binary_little_endian" : "ascii") << " 1.0\n";
			os << "element vertex " << num_vertices << "\n";
			os << "property float x\n";
			os << "property float y\n";
			os << "property float z\n";
			if (has_normals)
			{
				os << "property float nx\n";
				os << "property float ny\n";
				os << "property float nz\n";
			}
			os << "element face " << num_faces << "\n";
			os << "property list uchar int vertex_indices\n";
			os << "end_header\n";
		}

		void write_pointcloud_header(std::ostream& os, size_t num_points,
		                             bool has_normals, bool has_colors, bool binary)
		{
			os << "ply\n";
			os << "format " << (binary ? "binary_little_endian" : "ascii") << " 1.0\n";
			os << "element vertex " << num_points << "\n";
			os << "property float x\n";
			os << "property float y\n";
			os << "property float z\n";
			if (has_normals)
			{
				os << "property float nx\n";
				os << "property float ny\n";
				os << "property float nz\n";
			}
			if (has_colors)
			{
				os << "property uchar red\n";
				os << "property uchar green\n";
				os << "property uchar blue\n";
			}
			os << "end_header\n";
		}
	} // namespace

	bool PlyElement::has_property(std::string_view prop_name) const
	{
		return find_property(prop_name) != nullptr;
	}

	const PlyProperty* PlyElement::find_property(std::string_view prop_name) const
	{
		for (const auto& prop : properties)
		{
			if (prop.name == prop_name) return &prop;
		}
		return nullptr;
	}

	const PlyElement* PlyHeader::find_element(std::string_view name) const
	{
		for (const auto& elem : elements)
		{
			if (elem.name == name) return &elem;
		}
		return nullptr;
	}

	size_t PlyHeader::vertex_count() const
	{
		const auto* elem = find_element("vertex");
		return elem ? elem->count : 0;
	}

	size_t PlyHeader::face_count() const
	{
		const auto* elem = find_element("face");
		return elem ? elem->count : 0;
	}

	bool PlyHeader::has_normals() const
	{
		const auto* elem = find_element("vertex");
		return elem && (elem->has_property("nx") || elem->has_property("normal_x"));
	}

	bool PlyHeader::has_colors() const
	{
		const auto* elem = find_element("vertex");
		return elem && (elem->has_property("red") || elem->has_property("r"));
	}

	size_t PlyFile::type_size(PlyType type)
	{
		switch (type)
		{
		case PlyType::Char:
		case PlyType::UChar: return 1;
		case PlyType::Short:
		case PlyType::UShort: return 2;
		case PlyType::Int:
		case PlyType::UInt:
		case PlyType::Float: return 4;
		case PlyType::Double: return 8;
		case PlyType::Invalid:
			return 0;
		}
		return 0;
	}

	PlyType PlyFile::type_from_string(std::string_view str)
	{
		if (str == "char" || str == "int8") return PlyType::Char;
		if (str == "uchar" || str == "uint8") return PlyType::UChar;
		if (str == "short" || str == "int16") return PlyType::Short;
		if (str == "ushort" || str == "uint16") return PlyType::UShort;
		if (str == "int" || str == "int32") return PlyType::Int;
		if (str == "uint" || str == "uint32") return PlyType::UInt;
		if (str == "float" || str == "float32") return PlyType::Float;
		if (str == "double" || str == "float64") return PlyType::Double;
		return PlyType::Invalid;
	}

	core::Result<PlyHeader> PlyFile::read_header(const std::filesystem::path& path)
	{
		std::ifstream file(path, std::ios::binary);
		if (!file)
		{
			return core::unexpected(file_error(path, "Cannot open file"));
		}

		std::string line;
		if (!std::getline(file, line) || line.find("ply") == std::string::npos)
		{
			return core::unexpected(parse_error("Not a PLY file"));
		}

		PlyHeader header;
		PlyElement* current_element = nullptr;

		while (std::getline(file, line))
		{
			// 移除行尾 \r
			if (!line.empty() && line.back() == '\r') line.pop_back();
			if (line.find("end_header") != std::string::npos) break;

			std::istringstream iss(line);
			std::string keyword;
			iss >> keyword;

			if (keyword == "format")
			{
				std::string fmt;
				iss >> fmt;
				if (fmt == "ascii") header.format = PlyFormat::Ascii;
				else if (fmt == "binary_little_endian") header.format = PlyFormat::BinaryLittleEndian;
				else if (fmt == "binary_big_endian") header.format = PlyFormat::BinaryBigEndian;
			}
			else if (keyword == "comment")
			{
				std::string comment;
				std::getline(iss, comment);
				header.comments.push_back(comment);
			}
			else if (keyword == "element")
			{
				PlyElement elem;
				iss >> elem.name >> elem.count;
				header.elements.push_back(std::move(elem));
				current_element = &header.elements.back();
			}
			else if (keyword == "property" && current_element)
			{
				PlyProperty prop;
				std::string type_str;
				iss >> type_str;

				if (type_str == "list")
				{
					prop.is_list = true;
					std::string count_type_str, elem_type_str;
					iss >> count_type_str >> elem_type_str >> prop.name;
					prop.count_type = type_from_string(count_type_str);
					prop.type = type_from_string(elem_type_str);
				}
				else
				{
					prop.type = type_from_string(type_str);
					iss >> prop.name;
				}
				current_element->properties.push_back(std::move(prop));
			}
		}

		return header;
	}

	core::Result<mesh::Mesh> PlyFile::read_mesh(const std::filesystem::path& path)
	{
		auto header_result = read_header(path);
		if (!header_result) return core::unexpected(header_result.error());
		const auto& header = *header_result;

		std::ifstream file(path, std::ios::binary);
		if (!file)
		{
			return core::unexpected(file_error(path, "Cannot open file"));
		}

		// 跳过 header
		std::string line;
		while (std::getline(file, line))
		{
			if (line.find("end_header") != std::string::npos) break;
		}

		mesh::Mesh result;
		const size_t num_vertices = header.vertex_count();
		const size_t num_faces = header.face_count();
		const bool has_normals = header.has_normals();

		result.vertices.reserve(num_vertices);
		if (has_normals) result.normals.reserve(num_vertices);
		result.indices.reserve(num_faces * 3);

		if (header.format == PlyFormat::Ascii)
		{
			// ASCII 格式
			for (size_t i = 0; i < num_vertices; ++i)
			{
				if (!std::getline(file, line)) break;
				const char* p = line.data();
				const char* end = p + line.size();
				double x, y, z;
				if (!(p = parse_double(p, end, x))) continue;
				if (!(p = parse_double(p, end, y))) continue;
				if (!(p = parse_double(p, end, z))) continue;
				result.vertices.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
				if (has_normals)
				{
					double nx, ny, nz;
					if ((p = parse_double(p, end, nx)) &&
						(p = parse_double(p, end, ny)) &&
						(p = parse_double(p, end, nz)))
					{
						result.normals.emplace_back(static_cast<float>(nx), static_cast<float>(ny),
						                            static_cast<float>(nz));
					}
				}
			}
			for (size_t i = 0; i < num_faces; ++i)
			{
				if (!std::getline(file, line)) break;
				const char* p = line.data();
				const char* end = p + line.size();
				int count, i0, i1, i2;
				if (!(p = parse_int(p, end, count))) continue;
				if (count != 3) continue;
				if (!(p = parse_int(p, end, i0))) continue;
				if (!(p = parse_int(p, end, i1))) continue;
				if (!(p = parse_int(p, end, i2))) continue;
				result.indices.push_back(static_cast<uint32_t>(i0));
				result.indices.push_back(static_cast<uint32_t>(i1));
				result.indices.push_back(static_cast<uint32_t>(i2));
			}
		}
		else
		{
			// Binary: 一次性读入并行解析
			const size_t vertex_size = has_normals ? 6 * sizeof(float) : 3 * sizeof(float);
			constexpr size_t face_size = sizeof(uint8_t) + 3 * sizeof(int32_t);

			std::vector<char> vertex_buffer(num_vertices * vertex_size);
			file.read(vertex_buffer.data(), static_cast<std::streamsize>(vertex_buffer.size()));

			result.vertices.resize(num_vertices);
			if (has_normals) result.normals.resize(num_vertices);

INFMVS_OMP_PARALLEL_FOR
			for (int64_t i = 0; i < static_cast<int64_t>(num_vertices); ++i)
			{
				const char* ptr = vertex_buffer.data() + i * vertex_size;
				float x, y, z;
				std::memcpy(&x, ptr, sizeof(float));
				ptr += sizeof(float);
				std::memcpy(&y, ptr, sizeof(float));
				ptr += sizeof(float);
				std::memcpy(&z, ptr, sizeof(float));
				ptr += sizeof(float);
				result.vertices[i] = core::Vec3f(x, y, z);
				if (has_normals)
				{
					float nx, ny, nz;
					std::memcpy(&nx, ptr, sizeof(float));
					ptr += sizeof(float);
					std::memcpy(&ny, ptr, sizeof(float));
					ptr += sizeof(float);
					std::memcpy(&nz, ptr, sizeof(float));
					result.normals[i] = core::Vec3f(nx, ny, nz);
				}
			}

			std::vector<char> face_buffer(num_faces * face_size);
			file.read(face_buffer.data(), static_cast<std::streamsize>(face_buffer.size()));

			result.indices.resize(num_faces * 3);

			INFMVS_OMP_PARALLEL_FOR
			for (int64_t i = 0; i < static_cast<int64_t>(num_faces); ++i)
			{
				const char* ptr = face_buffer.data() + i * face_size;
				ptr += sizeof(uint8_t);
				int32_t i0, i1, i2;
				std::memcpy(&i0, ptr, sizeof(int32_t));
				ptr += sizeof(int32_t);
				std::memcpy(&i1, ptr, sizeof(int32_t));
				ptr += sizeof(int32_t);
				std::memcpy(&i2, ptr, sizeof(int32_t));
				result.indices[i * 3] = static_cast<uint32_t>(i0);
				result.indices[i * 3 + 1] = static_cast<uint32_t>(i1);
				result.indices[i * 3 + 2] = static_cast<uint32_t>(i2);
			}
		}

		return result;
	}

	core::Result<pc::PointCloud> PlyFile::read_pointcloud(const std::filesystem::path& path)
	{
		auto header_result = read_header(path);
		if (!header_result) return core::unexpected(header_result.error());
		const auto& header = *header_result;

		std::ifstream file(path, std::ios::binary);
		if (!file)
		{
			return core::unexpected(file_error(path, "Cannot open file"));
		}

		// 跳过 header
		std::string line;
		while (std::getline(file, line))
		{
			if (line.find("end_header") != std::string::npos) break;
		}

		pc::PointCloud result;
		const size_t num_points = header.vertex_count();
		const bool has_normals = header.has_normals();
		const bool has_colors = header.has_colors();

		// 检查颜色类型
		const auto* vertex_elem = header.find_element("vertex");
		bool color_is_float = false;
		if (vertex_elem)
		{
			const auto* red_prop = vertex_elem->find_property("red");
			if (!red_prop) red_prop = vertex_elem->find_property("r");
			if (red_prop) color_is_float = (red_prop->type == PlyType::Float || red_prop->type == PlyType::Double);
		}

		result.positions.reserve(num_points);
		if (has_normals) result.normals.reserve(num_points);
		if (has_colors) result.colors.reserve(num_points);

		if (header.format == PlyFormat::Ascii)
		{
			for (size_t i = 0; i < num_points; ++i)
			{
				if (!std::getline(file, line)) break;
				const char* p = line.data();
				const char* end = p + line.size();
				double x, y, z;
				if (!(p = parse_double(p, end, x))) continue;
				if (!(p = parse_double(p, end, y))) continue;
				if (!(p = parse_double(p, end, z))) continue;
				if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

				result.positions.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));

				if (has_normals)
				{
					double nx, ny, nz;
					if ((p = parse_double(p, end, nx)) && (p = parse_double(p, end, ny)) && (p = parse_double(
						p, end, nz)))
					{
						result.normals.emplace_back(static_cast<float>(nx), static_cast<float>(ny),
						                            static_cast<float>(nz));
					}
				}
				if (has_colors)
				{
					if (color_is_float)
					{
						double r, g, b;
						if ((p = parse_double(p, end, r)) && (p = parse_double(p, end, g)) && (p = parse_double(
							p, end, b)))
						{
							result.colors.emplace_back(static_cast<float>(r), static_cast<float>(g),
							                           static_cast<float>(b));
						}
					}
					else
					{
						int r, g, b;
						if ((p = parse_int(p, end, r)) && (p = parse_int(p, end, g)) && (p = parse_int(p, end, b)))
						{
							result.colors.emplace_back(static_cast<float>(r), static_cast<float>(g),
							                           static_cast<float>(b));
						}
					}
				}
			}
		}
		else
		{
			// Binary
			size_t point_size = 3 * sizeof(float);
			if (has_normals) point_size += 3 * sizeof(float);
			if (has_colors) point_size += color_is_float ? 3 * sizeof(float) : 3 * sizeof(uint8_t);

			std::vector<char> buffer(num_points * point_size);
			file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));

			result.positions.resize(num_points);
			if (has_normals) result.normals.resize(num_points);
			if (has_colors) result.colors.resize(num_points);

INFMVS_OMP_PARALLEL_FOR
			for (int64_t i = 0; i < static_cast<int64_t>(num_points); ++i)
			{
				const char* ptr = buffer.data() + i * point_size;
				float x, y, z;
				std::memcpy(&x, ptr, sizeof(float));
				ptr += sizeof(float);
				std::memcpy(&y, ptr, sizeof(float));
				ptr += sizeof(float);
				std::memcpy(&z, ptr, sizeof(float));
				ptr += sizeof(float);
				result.positions[i] = core::Vec3f(x, y, z);

				if (has_normals)
				{
					float nx, ny, nz;
					std::memcpy(&nx, ptr, sizeof(float));
					ptr += sizeof(float);
					std::memcpy(&ny, ptr, sizeof(float));
					ptr += sizeof(float);
					std::memcpy(&nz, ptr, sizeof(float));
					ptr += sizeof(float);
					result.normals[i] = core::Vec3f(nx, ny, nz);
				}
				if (has_colors)
				{
					if (color_is_float)
					{
						float r, g, b;
						std::memcpy(&r, ptr, sizeof(float));
						ptr += sizeof(float);
						std::memcpy(&g, ptr, sizeof(float));
						ptr += sizeof(float);
						std::memcpy(&b, ptr, sizeof(float));
						result.colors[i] = core::Vec3f(r, g, b);
					}
					else
					{
						uint8_t r, g, b;
						std::memcpy(&r, ptr, sizeof(uint8_t));
						ptr += sizeof(uint8_t);
						std::memcpy(&g, ptr, sizeof(uint8_t));
						ptr += sizeof(uint8_t);
						std::memcpy(&b, ptr, sizeof(uint8_t));
						result.colors[i] = core::Vec3f(r, g, b);
					}
				}
			}
		}

		return result;
	}

	core::Result<void> PlyFile::write_mesh(
		const std::filesystem::path& path, const mesh::Mesh& mesh, bool binary)
	{
		if (mesh.indices.size() % 3 != 0)
		{
			return core::unexpected(parse_error("Mesh indices must be triangles"));
		}
		if (mesh.vertices.empty())
		{
			return core::unexpected(parse_error("Mesh has no vertices"));
		}

		const bool has_normals = mesh.normals.size() == mesh.vertices.size();
		const size_t num_vertices = mesh.vertices.size();
		const size_t num_faces = mesh.indices.size() / 3;

		std::ofstream file(path, binary ? (std::ios::binary | std::ios::out) : std::ios::out);
		if (!file)
		{
			return core::unexpected(file_error(path, "Cannot open file for writing"));
		}

		write_mesh_header(file, num_vertices, num_faces, has_normals, binary);

		if (binary)
		{
			const size_t vertex_size = has_normals ? 6 * sizeof(float) : 3 * sizeof(float);
			constexpr size_t face_size = sizeof(uint8_t) + 3 * sizeof(int32_t);

			std::vector<char> vertex_buffer(num_vertices * vertex_size);
			std::vector<char> face_buffer(num_faces * face_size);

INFMVS_OMP_PARALLEL_FOR
			for (int64_t i = 0; i < static_cast<int64_t>(num_vertices); ++i)
			{
				char* ptr = vertex_buffer.data() + i * vertex_size;
				const auto& v = mesh.vertices[i];
				float x = v.x(), y = v.y(), z = v.z();
				std::memcpy(ptr, &x, sizeof(float));
				ptr += sizeof(float);
				std::memcpy(ptr, &y, sizeof(float));
				ptr += sizeof(float);
				std::memcpy(ptr, &z, sizeof(float));
				ptr += sizeof(float);
				if (has_normals)
				{
					const auto& n = mesh.normals[i];
					float nx = n.x(), ny = n.y(), nz = n.z();
					std::memcpy(ptr, &nx, sizeof(float));
					ptr += sizeof(float);
					std::memcpy(ptr, &ny, sizeof(float));
					ptr += sizeof(float);
					std::memcpy(ptr, &nz, sizeof(float));
				}
			}

INFMVS_OMP_PARALLEL_FOR
			for (int64_t i = 0; i < static_cast<int64_t>(num_faces); ++i)
			{
				char* ptr = face_buffer.data() + i * face_size;
				uint8_t count = 3;
				int32_t idx0 = static_cast<int32_t>(mesh.indices[i * 3]);
				int32_t idx1 = static_cast<int32_t>(mesh.indices[i * 3 + 1]);
				int32_t idx2 = static_cast<int32_t>(mesh.indices[i * 3 + 2]);
				std::memcpy(ptr, &count, sizeof(uint8_t));
				ptr += sizeof(uint8_t);
				std::memcpy(ptr, &idx0, sizeof(int32_t));
				ptr += sizeof(int32_t);
				std::memcpy(ptr, &idx1, sizeof(int32_t));
				ptr += sizeof(int32_t);
				std::memcpy(ptr, &idx2, sizeof(int32_t));
			}

			file.write(vertex_buffer.data(), static_cast<std::streamsize>(vertex_buffer.size()));
			file.write(face_buffer.data(), static_cast<std::streamsize>(face_buffer.size()));
		}
		else
		{
			// ASCII: 并行生成字符串
			const int num_threads = INFMVS_OMP_MAX_THREADS();
			const size_t chunk_v = (num_vertices + num_threads - 1) / num_threads;
			const size_t chunk_f = (num_faces + num_threads - 1) / num_threads;

			std::vector<std::string> vertex_chunks(num_threads);
			std::vector<std::string> face_chunks(num_threads);

			INFMVS_OMP_PARALLEL
			{
				const int tid = INFMVS_OMP_THREAD_NUM();
				const size_t start = tid * chunk_v;
				const size_t end = std::min(start + chunk_v, num_vertices);
				std::ostringstream oss;
				oss << std::fixed << std::setprecision(6);
				for (size_t i = start; i < end; ++i)
				{
					const auto& v = mesh.vertices[i];
					oss << v.x() << ' ' << v.y() << ' ' << v.z();
					if (has_normals)
					{
						const auto& n = mesh.normals[i];
						oss << ' ' << n.x() << ' ' << n.y() << ' ' << n.z();
					}
					oss << '\n';
				}
				vertex_chunks[tid] = oss.str();
			}

			INFMVS_OMP_PARALLEL
			{
				const int tid = INFMVS_OMP_THREAD_NUM();
				const size_t start = tid * chunk_f;
				const size_t end = std::min(start + chunk_f, num_faces);
				std::ostringstream oss;
				for (size_t i = start; i < end; ++i)
				{
					oss << "3 " << mesh.indices[i * 3] << ' '
						<< mesh.indices[i * 3 + 1] << ' '
						<< mesh.indices[i * 3 + 2] << '\n';
				}
				face_chunks[tid] = oss.str();
			}

			for (const auto& chunk : vertex_chunks) file << chunk;
			for (const auto& chunk : face_chunks) file << chunk;
		}

		return {};
	}

	core::Result<void> PlyFile::write_pointcloud(
		const std::filesystem::path& path, const pc::PointCloud& cloud, bool binary)
	{
		if (cloud.positions.empty())
		{
			return core::unexpected(parse_error("Point cloud has no points"));
		}

		const bool has_normals = !cloud.normals.empty() && cloud.normals.size() == cloud.positions.size();
		const bool has_colors = !cloud.colors.empty() && cloud.colors.size() == cloud.positions.size();
		const size_t num_points = cloud.positions.size();

		std::ofstream file(path, binary ? (std::ios::binary | std::ios::out) : std::ios::out);
		if (!file)
		{
			return core::unexpected(file_error(path, "Cannot open file for writing"));
		}

		write_pointcloud_header(file, num_points, has_normals, has_colors, binary);

		if (binary)
		{
			size_t point_size = 3 * sizeof(float);
			if (has_normals) point_size += 3 * sizeof(float);
			if (has_colors) point_size += 3 * sizeof(uint8_t);

			std::vector<char> buffer(num_points * point_size);

INFMVS_OMP_PARALLEL_FOR
			for (int64_t i = 0; i < static_cast<int64_t>(num_points); ++i)
			{
				char* ptr = buffer.data() + i * point_size;
				const auto& p = cloud.positions[i];
				float x = p.x(), y = p.y(), z = p.z();
				std::memcpy(ptr, &x, sizeof(float));
				ptr += sizeof(float);
				std::memcpy(ptr, &y, sizeof(float));
				ptr += sizeof(float);
				std::memcpy(ptr, &z, sizeof(float));
				ptr += sizeof(float);
				if (has_normals)
				{
					const auto& n = cloud.normals[i];
					float nx = n.x(), ny = n.y(), nz = n.z();
					std::memcpy(ptr, &nx, sizeof(float));
					ptr += sizeof(float);
					std::memcpy(ptr, &ny, sizeof(float));
					ptr += sizeof(float);
					std::memcpy(ptr, &nz, sizeof(float));
					ptr += sizeof(float);
				}
				if (has_colors)
				{
					const auto& c = cloud.colors[i];
					uint8_t r = clamp_color(c.x()), g = clamp_color(c.y()), b = clamp_color(c.z());
					std::memcpy(ptr, &r, sizeof(uint8_t));
					ptr += sizeof(uint8_t);
					std::memcpy(ptr, &g, sizeof(uint8_t));
					ptr += sizeof(uint8_t);
					std::memcpy(ptr, &b, sizeof(uint8_t));
				}
			}

			file.write(buffer.data(), static_cast<std::streamsize>(buffer.size()));
		}
		else
		{
			// ASCII: 并行生成
			const int num_threads = INFMVS_OMP_MAX_THREADS();
			const size_t chunk_size = (num_points + num_threads - 1) / num_threads;
			std::vector<std::string> chunks(num_threads);

			INFMVS_OMP_PARALLEL
			{
				const int tid = INFMVS_OMP_THREAD_NUM();
				const size_t start = tid * chunk_size;
				const size_t end = std::min(start + chunk_size, num_points);
				std::ostringstream oss;
				oss << std::fixed << std::setprecision(6);
				for (size_t i = start; i < end; ++i)
				{
					const auto& p = cloud.positions[i];
					oss << p.x() << ' ' << p.y() << ' ' << p.z();
					if (has_normals)
					{
						const auto& n = cloud.normals[i];
						oss << ' ' << n.x() << ' ' << n.y() << ' ' << n.z();
					}
					if (has_colors)
					{
						const auto& c = cloud.colors[i];
						oss << ' ' << static_cast<int>(clamp_color(c.x()))
							<< ' ' << static_cast<int>(clamp_color(c.y()))
							<< ' ' << static_cast<int>(clamp_color(c.z()));
					}
					oss << '\n';
				}
				chunks[tid] = oss.str();
			}

			for (const auto& chunk : chunks) file << chunk;
		}

		return {};
	}
} // namespace inf::io
