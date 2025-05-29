return {
	"nvim-java/nvim-java",
	priority = 100, -- Crucial to load before lspconfig
	dependencies = {
		{ "neovim/nvim-lspconfig" },
	},
}
