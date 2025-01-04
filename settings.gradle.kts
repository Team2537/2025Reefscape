pluginManagement {
    repositories {
        mavenLocal()
        gradlePluginPortal()
        val frcYear = "2025"
        val frcHome = if (System.getProperty("os.name").contains("windows", ignoreCase = true)) {
            val publicFolder = System.getenv("PUBLIC") ?: """C:\Users\Public"""
            val homeRoot = File(publicFolder, "wpilib")
            File(homeRoot, frcYear)
        } else {
            val userFolder = System.getProperty("user.home")
            val homeRoot = File(userFolder, "wpilib")
            File(homeRoot, frcYear)
        }
        val frcHomeMaven = File(frcHome, "maven")
        maven {
            name = "frcHome"
            setUrl(frcHomeMaven)
        }
    }
}

System.getProperties().apply {
    setProperty("org.gradle.internal.native.headers.unresolved.dependencies.ignore", "true")
}

plugins {
    id("org.gradle.toolchains.foojay-resolver-convention") version ("0.9.0")
}
